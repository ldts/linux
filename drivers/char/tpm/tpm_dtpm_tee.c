// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * Driver for a discrete TPM that is owned by OP-TEE and exposed to the
 * normal world through a pseudo TA. Unlike the firmware TPM (tpm_ftpm_tee),
 * the TPM here is a real chip on a bus that only the secure world can reach
 * (e.g. an SPI TPM behind OP-TEE's TPM TIS driver). OP-TEE advertises a
 * passthrough service on the TEE device bus; this driver binds to it by
 * UUID and pipes raw TPM2 commands through, presenting a standard tpm_chip.
 *
 * The passthrough command shape mirrors the fTPM TA's SUBMIT_COMMAND
 * (memref[0] = command in, memref[1] = response in/out) but uses a distinct
 * UUID rather than the fTPM identity.
 */

#include <linux/module.h>
#include <linux/tee_drv.h>
#include <linux/tpm.h>
#include <linux/uuid.h>

#include "tpm.h"

/*
 * PTA_TPM_PASSTHROUGH_UUID: 50568cd9-370a-46b8-9f2c-6fd90664d2ee
 *
 * Must match the UUID of the discrete TPM passthrough pseudo TA in OP-TEE.
 */
static const uuid_t dtpm_ta_uuid =
	UUID_INIT(0x50568cd9, 0x370a, 0x46b8,
		  0x9f, 0x2c, 0x6f, 0xd9, 0x06, 0x64, 0xd2, 0xee);

/* Command IDs implemented by the passthrough TA */
#define DTPM_TEE_TA_SUBMIT_COMMAND	0

/* Max buffer sizes, matching the TPM2 command/response limits */
#define DTPM_MAX_COMMAND_SIZE		4096
#define DTPM_MAX_RESPONSE_SIZE		4096

/**
 * struct dtpm_tee_private - discrete TPM passthrough private data
 * @chip:     struct tpm_chip instance registered with the tpm framework
 * @session:  passthrough TA session identifier
 * @ctx:      TEE context handler
 * @shm:      memory pool shared with the passthrough TA in TEE
 */
struct dtpm_tee_private {
	struct tpm_chip *chip;
	u32 session;
	struct tee_context *ctx;
	struct tee_shm *shm;
};

/**
 * dtpm_tee_tpm_op_send() - send a TPM command through TEE shared memory and
 * retrieve the response.
 * @chip:	the tpm_chip description
 * @buf:	buffer holding the command on entry, the response on return
 * @bufsiz:	size of @buf
 * @cmd_len:	number of command bytes to send
 *
 * Return: number of response bytes on success, -errno on failure.
 */
static int dtpm_tee_tpm_op_send(struct tpm_chip *chip, u8 *buf, size_t bufsiz,
				size_t cmd_len)
{
	struct dtpm_tee_private *pvt_data = dev_get_drvdata(chip->dev.parent);
	size_t resp_len;
	int rc;
	u8 *temp_buf;
	struct tpm_header *resp_header;
	struct tee_ioctl_invoke_arg transceive_args;
	struct tee_param command_params[4];
	struct tee_shm *shm = pvt_data->shm;

	if (cmd_len > DTPM_MAX_COMMAND_SIZE) {
		dev_err(&chip->dev, "command length %zu exceeds %u\n",
			cmd_len, DTPM_MAX_COMMAND_SIZE);
		return -EIO;
	}

	memset(&transceive_args, 0, sizeof(transceive_args));
	memset(command_params, 0, sizeof(command_params));

	/* Invoke the SUBMIT_COMMAND function of the passthrough TA */
	transceive_args = (struct tee_ioctl_invoke_arg) {
		.func = DTPM_TEE_TA_SUBMIT_COMMAND,
		.session = pvt_data->session,
		.num_params = 4,
	};

	/* memref[0]: command input */
	command_params[0] = (struct tee_param) {
		.attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT,
		.u.memref = {
			.shm = shm,
			.size = cmd_len,
			.shm_offs = 0,
		},
	};

	temp_buf = tee_shm_get_va(shm, 0);
	if (IS_ERR(temp_buf)) {
		dev_err(&chip->dev, "tee_shm_get_va failed for transmit\n");
		return PTR_ERR(temp_buf);
	}
	memset(temp_buf, 0, (DTPM_MAX_COMMAND_SIZE + DTPM_MAX_RESPONSE_SIZE));
	memcpy(temp_buf, buf, cmd_len);

	/* memref[1]: response in/out */
	command_params[1] = (struct tee_param) {
		.attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT,
		.u.memref = {
			.shm = shm,
			.size = DTPM_MAX_RESPONSE_SIZE,
			.shm_offs = DTPM_MAX_COMMAND_SIZE,
		},
	};

	rc = tee_client_invoke_func(pvt_data->ctx, &transceive_args,
				    command_params);
	if (rc < 0 || transceive_args.ret != 0) {
		dev_err(&chip->dev, "SUBMIT_COMMAND invoke error: 0x%x\n",
			transceive_args.ret);
		return (rc < 0) ? rc : -EIO;
	}

	temp_buf = tee_shm_get_va(shm, command_params[1].u.memref.shm_offs);
	if (IS_ERR(temp_buf)) {
		dev_err(&chip->dev, "tee_shm_get_va failed for receive\n");
		return PTR_ERR(temp_buf);
	}

	resp_header = (struct tpm_header *)temp_buf;
	resp_len = be32_to_cpu(resp_header->length);

	/* sanity check resp_len */
	if (resp_len < TPM_HEADER_SIZE) {
		dev_err(&chip->dev, "TPM response header too small\n");
		return -EIO;
	}
	if (resp_len > DTPM_MAX_RESPONSE_SIZE) {
		dev_err(&chip->dev, "response length %zu exceeds %u\n",
			resp_len, DTPM_MAX_RESPONSE_SIZE);
		return -EIO;
	}
	if (resp_len > bufsiz) {
		dev_err(&chip->dev, "response length %zu exceeds buffer size %zu\n",
			resp_len, bufsiz);
		return -EIO;
	}

	memcpy(buf, temp_buf, resp_len);

	return resp_len;
}

static const struct tpm_class_ops dtpm_tee_tpm_ops = {
	.flags = TPM_OPS_AUTO_STARTUP,
	.send = dtpm_tee_tpm_op_send,
};

/*
 * Check whether this driver supports the passthrough TA in the TEE instance
 * represented by @ver.
 */
static int dtpm_tee_match(struct tee_ioctl_version_data *ver, const void *data)
{
	/* Only GP-compliant OP-TEE is supported */
	if (ver->impl_id == TEE_IMPL_ID_OPTEE &&
	    (ver->gen_caps & TEE_GEN_CAP_GP))
		return 1;

	return 0;
}

static int dtpm_tee_probe(struct tee_client_device *tcdev)
{
	struct device *dev = &tcdev->dev;
	struct dtpm_tee_private *pvt_data;
	struct tee_ioctl_open_session_arg sess_arg;
	struct tpm_chip *chip;
	int rc;

	pvt_data = devm_kzalloc(dev, sizeof(*pvt_data), GFP_KERNEL);
	if (!pvt_data)
		return -ENOMEM;

	dev_set_drvdata(dev, pvt_data);

	/* Open context with TEE driver */
	pvt_data->ctx = tee_client_open_context(NULL, dtpm_tee_match, NULL,
						NULL);
	if (IS_ERR(pvt_data->ctx)) {
		if (PTR_ERR(pvt_data->ctx) == -ENOENT)
			return -EPROBE_DEFER;
		dev_err(dev, "tee_client_open_context failed\n");
		return PTR_ERR(pvt_data->ctx);
	}

	/*
	 * Open a session with the passthrough TA as the REE kernel. The TA
	 * gates access to the shared TPM on TEE_LOGIN_REE_KERNEL so that REE
	 * user space and other TAs cannot submit raw TPM commands.
	 */
	memset(&sess_arg, 0, sizeof(sess_arg));
	export_uuid(sess_arg.uuid, &dtpm_ta_uuid);
	sess_arg.clnt_login = TEE_IOCTL_LOGIN_REE_KERNEL;
	sess_arg.num_params = 0;

	rc = tee_client_open_session(pvt_data->ctx, &sess_arg, NULL);
	if (rc < 0 || sess_arg.ret != 0) {
		dev_err(dev, "tee_client_open_session failed, err=0x%x\n",
			sess_arg.ret);
		rc = -EINVAL;
		goto out_tee_session;
	}
	pvt_data->session = sess_arg.session;

	/* Allocate dynamic shared memory with the passthrough TA */
	pvt_data->shm = tee_shm_alloc_kernel_buf(pvt_data->ctx,
						 DTPM_MAX_COMMAND_SIZE +
						 DTPM_MAX_RESPONSE_SIZE);
	if (IS_ERR(pvt_data->shm)) {
		dev_err(dev, "tee_shm_alloc_kernel_buf failed\n");
		rc = -ENOMEM;
		goto out_shm_alloc;
	}

	/* Allocate a struct tpm_chip instance */
	chip = tpm_chip_alloc(dev, &dtpm_tee_tpm_ops);
	if (IS_ERR(chip)) {
		dev_err(dev, "tpm_chip_alloc failed\n");
		rc = PTR_ERR(chip);
		goto out_chip_alloc;
	}

	pvt_data->chip = chip;
	pvt_data->chip->flags |= TPM_CHIP_FLAG_TPM2 | TPM_CHIP_FLAG_SYNC;

	rc = tpm_chip_register(pvt_data->chip);
	if (rc) {
		dev_err(dev, "tpm_chip_register failed with rc=%d\n", rc);
		goto out_chip;
	}

	return 0;

out_chip:
	put_device(&pvt_data->chip->dev);
out_chip_alloc:
	tee_shm_free(pvt_data->shm);
out_shm_alloc:
	tee_client_close_session(pvt_data->ctx, pvt_data->session);
out_tee_session:
	tee_client_close_context(pvt_data->ctx);

	return rc;
}

static void dtpm_tee_remove(struct tee_client_device *tcdev)
{
	struct dtpm_tee_private *pvt_data = dev_get_drvdata(&tcdev->dev);

	/* Release the chip */
	tpm_chip_unregister(pvt_data->chip);

	/* frees chip */
	put_device(&pvt_data->chip->dev);

	/* Free the shared memory pool */
	tee_shm_free(pvt_data->shm);

	/* Close the session with the passthrough TA */
	tee_client_close_session(pvt_data->ctx, pvt_data->session);

	/* Close the context with the TEE driver */
	tee_client_close_context(pvt_data->ctx);

	/* memory allocated with devm_kzalloc() is freed automatically */
}

static const struct tee_client_device_id dtpm_tee_id_table[] = {
	{UUID_INIT(0x50568cd9, 0x370a, 0x46b8,
		   0x9f, 0x2c, 0x6f, 0xd9, 0x06, 0x64, 0xd2, 0xee)},
	{}
};
MODULE_DEVICE_TABLE(tee, dtpm_tee_id_table);

static struct tee_client_driver dtpm_tee_driver = {
	.probe		= dtpm_tee_probe,
	.remove		= dtpm_tee_remove,
	.id_table	= dtpm_tee_id_table,
	.driver		= {
		.name		= "optee-dtpm",
	},
};

static int __init dtpm_tee_mod_init(void)
{
	return tee_client_driver_register(&dtpm_tee_driver);
}

static void __exit dtpm_tee_mod_exit(void)
{
	tee_client_driver_unregister(&dtpm_tee_driver);
}

module_init(dtpm_tee_mod_init);
module_exit(dtpm_tee_mod_exit);

MODULE_DESCRIPTION("TPM Driver for a discrete TPM behind OP-TEE");
MODULE_LICENSE("GPL");
