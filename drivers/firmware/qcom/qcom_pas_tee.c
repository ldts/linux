// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/delay.h>
#include <linux/of.h>
#include <linux/firmware/qcom/qcom_pas.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/tee_drv.h>
#include <linux/uuid.h>

#include "qcom_pas.h"

#define DRIVER_NAME "qcom-pas-tee"

/*
 * Peripheral Authentication Service (PAS) supported.
 *
 * Get PAS support status.
 *
 * [in]  params[0].value.a:	Unique 32bit remote processor identifier
 * [out] params[1].value.a:	Firmware format (PTA_RPROC_HWCAP_FMT_*)
 * [out] params[2].value.a:	Image protection method (PTA_RPROC_HWCAP_PROT_*)
 */
#define PTA_QCOM_PAS_IS_SUPPORTED		1

/*
 * PAS capabilities.
 *
 * Get PAS capabilities.
 *
 * [in]  params[0].value.a:	Unique 32bit remote processor identifier
 * [out] params[1].value.a:	Firmware format (PTA_RPROC_HWCAP_FMT_*)
 * [out] params[2].value.a:	Image protection method (PTA_RPROC_HWCAP_PROT_*)
 */
#define PTA_QCOM_PAS_CAPABILITIES		2

/*
 * PAS image initialization.
 *
 * Perform PAS image initialization.
 *
 * [in]  params[0].value.a:	Unique 32bit remote processor identifier
 * [in]  params[1].memref:	Loadable firmware image
 */
#define PTA_QCOM_PAS_INIT_IMAGE			3

/*
 * PAS memory setup.
 *
 * Perform PAS memory setup.
 *
 * [in]  params[0].value.a:	Unique 32bit remote processor identifier
 * [in]  params[1].memref:	Section data to load
 * [in]  params[2].value.a:	32bit LSB load device segment address
 * [in]  params[2].value.b:	32bit MSB load device segment address
 * [in]  params[3].memref:	Expected hash (SHA256) of the payload
 */
#define PTA_QCOM_PAS_MEM_SETUP			4

/*
 * PAS image authentication and co-processor reset.
 *
 * Perform PAS image authentication and co-processor reset.
 *
 * [in]  params[0].value.a:	Unique 32bit remote processor identifier
 * [in]  params[1].value.a:	32bit LSB device memory address
 * [in]  params[1].value.b:	32bit MSB device memory address
 * [in]  params[2].value.a:	32bit LSB device memory size
 * [in]  params[2].value.b:	32bit MSB device memory size
 * [in]  params[3].value.a:	Byte value to be set
 */
#define PTA_QCOM_PAS_AUTH_AND_RESET		5

/*
 * PAS co-processor shutdown.
 *
 * Perform PAS co-processor shutdown.
 *
 * [in]  params[0].value.a:	Unique 32bit remote processor identifier
 */
#define PTA_QCOM_PAS_SHUTDOWN			6

/**
 * struct qcom_pas_tee_private - PAS service private data
 * @dev:		PAS service device.
 * @ctx:		TEE context handler.
 * @session_id:		PAS TA session identifier.
 */
struct qcom_pas_tee_private {
	struct device *dev;
	struct tee_context *ctx;
	u32 session_id;
};

static bool qcom_pas_tee_supported(struct device *dev, u32 peripheral)
{
	struct qcom_pas_tee_private *data = dev_get_drvdata(dev);
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];
	int ret = 0;

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	inv_arg.func = PTA_QCOM_PAS_IS_SUPPORTED;
	inv_arg.session = data->session_id;
	inv_arg.num_params = 4;

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[0].u.value.a = peripheral;

	ret = tee_client_invoke_func(data->ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0)) {
		dev_dbg(dev, "PAS not supported, peripheral: %d, err: %x\n",
			peripheral, inv_arg.ret);
		return false;
	}

	return true;
}

static int qcom_pas_tee_init_image(struct device *dev, u32 peripheral,
				   const void *metadata, size_t size,
				   struct qcom_pas_metadata *ctx)
{
	struct qcom_pas_tee_private *data = dev_get_drvdata(dev);
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_shm *mdata_shm = NULL;
	struct tee_param param[4];
	u8 *mdata_buf = NULL;
	int ret = 0, err = -ENODEV;

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	mdata_shm = tee_shm_alloc_kernel_buf(data->ctx, size);
	if (IS_ERR(mdata_shm)) {
		dev_err(dev, "mdata_shm allocation failed\n");
		return PTR_ERR(mdata_shm);
	}

	mdata_buf = tee_shm_get_va(mdata_shm, 0);
	if (IS_ERR(mdata_buf)) {
		dev_err(dev, "mdata_buf get VA failed\n");
		err = PTR_ERR(mdata_buf);
		goto err_shm;
	}
	memcpy(mdata_buf, metadata, size);

	inv_arg.func = PTA_QCOM_PAS_INIT_IMAGE;
	inv_arg.session = data->session_id;
	inv_arg.num_params = 4;

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[0].u.value.a = peripheral;
	param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
	param[1].u.memref.shm = mdata_shm;
	param[1].u.memref.size = size;
	param[1].u.memref.shm_offs = 0;

	ret = tee_client_invoke_func(data->ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0)) {
		dev_err(dev, "PAS init image failed, peripheral: %d, err: %x\n",
			peripheral, inv_arg.ret);
		err = -EINVAL;
		goto err_shm;
	}

	if (ctx)
		ctx->ptr = (void *)mdata_shm;

	return 0;
err_shm:
	tee_shm_free(mdata_shm);

	return err;
}

static int qcom_pas_tee_mem_setup(struct device *dev, u32 peripheral,
				  phys_addr_t addr, phys_addr_t size)
{
	struct qcom_pas_tee_private *data = dev_get_drvdata(dev);
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];
	int ret = 0;

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	inv_arg.func = PTA_QCOM_PAS_MEM_SETUP;
	inv_arg.session = data->session_id;
	inv_arg.num_params = 4;

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[0].u.value.a = peripheral;
	param[0].u.value.b = addr;
	param[0].u.value.c = size;

	ret = tee_client_invoke_func(data->ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0)) {
		dev_err(dev, "PAS mem setup failed, peripheral: %d, err: %x\n",
			peripheral, inv_arg.ret);
		return -EINVAL;
	}

	return 0;
}

static int qcom_pas_tee_auth_and_reset(struct device *dev, u32 peripheral)
{
	struct qcom_pas_tee_private *data = dev_get_drvdata(dev);
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];
	int ret = 0;

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	inv_arg.func = PTA_QCOM_PAS_AUTH_AND_RESET;
	inv_arg.session = data->session_id;
	inv_arg.num_params = 4;

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[0].u.value.a = peripheral;

	ret = tee_client_invoke_func(data->ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0)) {
		dev_err(dev, "PAS auth reset failed, peripheral: %d, err: %x\n",
			peripheral, inv_arg.ret);
		return -EINVAL;
	}

	return 0;
}

static int qcom_pas_tee_shutdown(struct device *dev, u32 peripheral)
{
	struct qcom_pas_tee_private *data = dev_get_drvdata(dev);
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];
	int ret = 0;

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	inv_arg.func = PTA_QCOM_PAS_SHUTDOWN;
	inv_arg.session = data->session_id;
	inv_arg.num_params = 4;

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[0].u.value.a = peripheral;

	ret = tee_client_invoke_func(data->ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0)) {
		dev_err(dev, "PAS shutdown failed, peripheral: %d, err: %x\n",
			peripheral, inv_arg.ret);
		return -EINVAL;
	}

	return 0;
}

static void qcom_pas_tee_metadata_release(struct device *dev,
				   struct qcom_pas_metadata *ctx)
{
	struct tee_shm *mdata_shm = ctx->ptr;

	tee_shm_free(mdata_shm);
}

static struct qcom_pas_ops qcom_pas_ops_tee = {
	.drv_name	= DRIVER_NAME,
	.supported	= qcom_pas_tee_supported,
	.init_image	= qcom_pas_tee_init_image,
	.mem_setup	= qcom_pas_tee_mem_setup,
	.auth_and_reset	= qcom_pas_tee_auth_and_reset,
	.shutdown	= qcom_pas_tee_shutdown,
	.metadata_release = qcom_pas_tee_metadata_release,
};

static int optee_ctx_match(struct tee_ioctl_version_data *ver, const void *data)
{
	if (ver->impl_id == TEE_IMPL_ID_OPTEE)
		return 1;
	else
		return 0;
}

static int qcom_pas_tee_probe(struct device *dev)
{
	struct qcom_pas_tee_private *data;
	struct tee_client_device *pas_device = to_tee_client_device(dev);
	struct tee_ioctl_open_session_arg sess_arg;
	int ret = 0, err = -ENODEV;

	dev_err(dev, "qcom_pas_tee_probe entered");
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->ctx = tee_client_open_context(NULL, optee_ctx_match, NULL, NULL);
	if (IS_ERR(data->ctx)) {
		err = -ENODEV;
		goto out_data;
	}

	memset(&sess_arg, 0, sizeof(sess_arg));
	export_uuid(sess_arg.uuid, &pas_device->id.uuid);
	sess_arg.clnt_login = TEE_IOCTL_LOGIN_REE_KERNEL;
	sess_arg.num_params = 0;

	ret = tee_client_open_session(data->ctx, &sess_arg, NULL);
	if ((ret < 0) || (sess_arg.ret != 0)) {
		dev_err(dev, "tee_client_open_session failed, err: %x\n",
			sess_arg.ret);
		err = -EINVAL;
		goto out_ctx;
	}
	data->session_id = sess_arg.session;
	dev_set_drvdata(dev, data);
	qcom_pas_ops_tee.dev = dev;

	qcom_pas_ops_register(&qcom_pas_ops_tee);
	dev_err(dev, "qcom_pas_tee_probe exit");

	return 0;

out_ctx:
	tee_client_close_context(data->ctx);
out_data:
	kfree(data);

	return err;
}

static int qcom_pas_tee_remove(struct device *dev)
{
	struct qcom_pas_tee_private *data = dev_get_drvdata(dev);

	tee_client_close_session(data->ctx, data->session_id);
	tee_client_close_context(data->ctx);

	return 0;
}

static const struct tee_client_device_id qcom_pas_tee_id_table[] = {
	{UUID_INIT(0xcff7d191, 0x7ca0, 0x4784,
		   0xaf, 0x13, 0x48, 0x22, 0x3b, 0x9a, 0x4f, 0xbe)},
	{}
};

MODULE_DEVICE_TABLE(tee, qcom_pas_tee_id_table);

static struct tee_client_driver qcom_pas_tee_driver = {
	.id_table	= qcom_pas_tee_id_table,
	.driver		= {
		.name		= DRIVER_NAME,
		.bus		= &tee_bus_type,
		.probe		= qcom_pas_tee_probe,
		.remove		= qcom_pas_tee_remove,
	},
};

static int __init qcom_pas_tee_mod_init(void)
{
	return driver_register(&qcom_pas_tee_driver.driver);
}

static void __exit qcom_pas_tee_mod_exit(void)
{
	driver_unregister(&qcom_pas_tee_driver.driver);
}

module_init(qcom_pas_tee_mod_init);
module_exit(qcom_pas_tee_mod_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sumit Garg <sumit.garg@oss.qualcomm.com>");
MODULE_DESCRIPTION("TEE bus based Qualcomm PAS driver");
