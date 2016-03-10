/*
 *    Based on code from Cortina Systems, Inc.
 *
 *    Copyright (C) 2011, 2012 by Cortina Systems, Inc.
 *    Copyright (C) 2011, 2012 Cavium, Inc.
 *    Copyright (C) 2014-2016 Skyport Systems, Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/phy.h>
#include <linux/of.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

#define CS4321_GLOBAL_CHIP_ID_LSB			0x0
#define CS4321_GLOBAL_CHIP_ID_MSB			0x1
#define CS4321_GPIO_GPIO_INPUT_INTS			0x169
#define CS4321_GPIO_GPIO_INTS				0x16D
#define CS4321_DSP_SDS_TEMPMON_MON_CONTROL0		0x440
#define CS4321_DSP_SDS_TEMPMON_MON_CONTROL1		0x441
#define CS4321_DSP_SDS_TEMPMON_MON_STATUS2		0x448
#define CS4321_DSP_SDS_TEMPMON_MON_LUT_RANGE0		0x44F
#define CS4321_DSP_SDS_TEMPMON_MON_LUT_VALUE0		0x45F
#define CS4321_LINE_SDS_COMMON_SRX0_RX_CONFIG		0x500
#define CS4321_GIGEPCS_INT_LINE_PCS1GE_INTSTATUS	0xC42
#define CS4321_SFP_CACHE_CONFIG_STATUS			0x3701
#define CS4321_SFP_CACHE_MODDEF_CACHE			0x3800
#define CS4321_SFP_CACHE_DIAG_CACHE			0x3900

#define CS4321_CHIP_ID_LSB				0x23E5
#define CS4321_CHIP_ID_MSB_A0				0x1002
#define CS4321_CHIP_ID_MSB_B0				0x2002

struct cs4321_private {
	bool gige_mode;
};

static int cs4321_phy_read(struct phy_device *phydev, u16 regnum)
{
	return mdiobus_read(phydev->bus, phydev->addr, MII_ADDR_C45 | regnum);
}

static int cs4321_phy_write(struct phy_device *phydev, u16 regnum, u16 val)
{
	return mdiobus_write(phydev->bus, phydev->addr, MII_ADDR_C45 | regnum,
			     val);
}

static inline uint16_t cs4321_twos_complement(uint16_t input)
{
	return ((input & 0x8000) == 0) ? input : 65536 - (input & 0x7fff);
}

static int cs4321_hsif_mon_temp_program_lut(struct phy_device *phydev)
{
	int i;
	static const uint16_t cs4321_lut_mon_temp[] = {
	    0x6059, 0x81FD, /* Index 0 -> DSP_SDS_TEMPMON_MON_LUT_RANGE0,VALUE0 */
	    0x685E, 0x82E2, /* Index 1 -> DSP_SDS_TEMPMON_MON_LUT_RANGE1,VALUE1 */
	    0x6E9A, 0x826F, /* Index 2 */
	    0x74F9, 0x81FC, /* Index 3 */
	    0x7B79, 0x81CD, /* Index 4 */
	    0x8235, 0x819F, /* Index 5 */
	    0x892D, 0x8114, /* Index 6 */
	    0x9026, 0x808A, /* Index 7 */
	    0xFFFF, 0x0000, /* Index 8 */
	    0x0000, 0x0000, /* Index 9 */
	    0x0000, 0x0000, /* Index 10 */
	    0x0000, 0x0000, /* Index 11 */
	    0x0000, 0x0000, /* Index 12 */
	    0x0000, 0x0000, /* Index 13 */
	    0x0000, 0x0000, /* Index 14 */
	    0x0000, 0x0000, /* Index 15 -> DSP_SDS_TEMPMON_MON_LUT_RANGE15,VALUE15*/
	};
	for (i = 0; i < 32; i += 2) {
		uint16_t range = cs4321_lut_mon_temp[i];
		uint16_t value = cs4321_twos_complement(cs4321_lut_mon_temp[i + 1]);
		int index = i / 2;
		cs4321_phy_write(phydev,
				 CS4321_DSP_SDS_TEMPMON_MON_LUT_RANGE0 + index,
				 range);
		cs4321_phy_write(phydev,
				 CS4321_DSP_SDS_TEMPMON_MON_LUT_VALUE0 + index,
				 value);
	}
	return 0;
}

static ssize_t cs4321_show_temp(struct device *dev,
				struct device_attribute *devattr, char *buf)
{
	struct phy_device *phydev = to_phy_device(dev);
	int data;
	int temp = 0;

	data = cs4321_phy_read(phydev, CS4321_LINE_SDS_COMMON_SRX0_RX_CONFIG);
	if (data < 0)
		goto done;
	data &= ~0x2000;
	cs4321_phy_write(phydev, CS4321_LINE_SDS_COMMON_SRX0_RX_CONFIG, data);
	udelay(10);

	cs4321_phy_write(phydev, CS4321_DSP_SDS_TEMPMON_MON_CONTROL0, 0x0333);
	cs4321_phy_write(phydev, CS4321_DSP_SDS_TEMPMON_MON_CONTROL1, 0x0010);
	cs4321_hsif_mon_temp_program_lut(phydev);
	mdelay(1);

	data = cs4321_phy_read(phydev, CS4321_DSP_SDS_TEMPMON_MON_STATUS2);
	if (data < 0)
		goto done;

	/* This is the equivalent of ((data / 256.0) * -1.122) + 210.37 */
	temp = ((data * -1122) + (256 * 210370)) / 256;

 done:
	return sprintf(buf, "%d\n", temp);
}

static ssize_t cs4321_show_name(struct device *dev,
				struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "cs4321\n");
}

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, cs4321_show_temp, NULL, 0);
static SENSOR_DEVICE_ATTR(name, S_IRUGO, cs4321_show_name, NULL, 0);

static struct attribute *cs4321_attributes[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_name.dev_attr.attr,
	NULL
};

static const struct attribute_group cs4321_attribute_group = {
	.attrs = cs4321_attributes,
};

static ssize_t do_sfp_read(struct phy_device *phydev, int type,
			   char *buf, loff_t off, size_t count)
{
	static struct {
		int cache_off;
		int stat_mask;
		int stat_good;
	} param[2] = {
		{ CS4321_SFP_CACHE_MODDEF_CACHE, 0x0025, 0x0001 },
		{ CS4321_SFP_CACHE_DIAG_CACHE, 0x001a, 0x0002 },
	};
	int i;

	if (cs4321_phy_read(phydev, CS4321_GPIO_GPIO_INPUT_INTS) & 0x0010) {
		/* mod_abs asserted */
		return -ENODEV;
	}

	/* Wait up to 2 sec for automatic refresh of SFP cache */
	for (i = 0; i < 10; i++) {
		if ((cs4321_phy_read(phydev, CS4321_SFP_CACHE_CONFIG_STATUS)
		     & param[type].stat_mask) == param[type].stat_good)
			break;
		msleep(200);
	}
	if (i >= 10)
		return -ETIMEDOUT;

	for (i = 0; i < count; i++) {
		int data = cs4321_phy_read(phydev,
					   param[type].cache_off + off + i);
		if (data < 0)
			return -EINVAL;
		buf[i] = data;
	}

	return count;
}

static ssize_t cs4321_sfp_read(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr,
		char *buf, loff_t off, size_t count)
{
	return do_sfp_read(to_phy_device(container_of(kobj, struct device, kobj)),
			   (uintptr_t) attr->private,
			   buf, off, count);
}

static struct bin_attribute cs4321_sfp_ident_attr = {
	.attr = {
		.name = "sfp_ident",
		.mode = S_IRUSR,
	},
	.size = 256,
	.read = cs4321_sfp_read,
	.private = (void *) 0,
};

static struct bin_attribute cs4321_sfp_diag_attr = {
	.attr = {
		.name = "sfp_diag",
		.mode = S_IRUSR,
	},
	.size = 256,
	.read = cs4321_sfp_read,
	.private = (void *) 1,
};

static int cs4321_config_init(struct phy_device *phydev)
{
	phydev->state = PHY_NOLINK;
	return 0;
}

static int cs4321_soft_reset(struct phy_device *phydev)
{
	return 0;
}

static void cs4321_remove(struct phy_device *phydev)
{
	sysfs_remove_bin_file(&phydev->dev.kobj, &cs4321_sfp_diag_attr);
	sysfs_remove_bin_file(&phydev->dev.kobj, &cs4321_sfp_ident_attr);
	hwmon_device_unregister(&phydev->dev);
	sysfs_remove_group(&phydev->dev.kobj, &cs4321_attribute_group);
	kfree(phydev->priv);
}

static int cs4321_probe(struct phy_device *phydev)
{
	int ret = 0;
	int id_lsb, id_msb;
	bool gige_mode;
	const char *prop_val;
	struct cs4321_private *p;

	id_lsb = cs4321_phy_read(phydev, CS4321_GLOBAL_CHIP_ID_LSB);
	id_msb = cs4321_phy_read(phydev, CS4321_GLOBAL_CHIP_ID_MSB);
	if (id_lsb != CS4321_CHIP_ID_LSB ||
	    (id_msb != CS4321_CHIP_ID_MSB_A0 && id_msb != CS4321_CHIP_ID_MSB_B0)) {
		return -ENODEV;
	}

	ret = of_property_read_string(phydev->dev.of_node, "cortina,host-mode",
				      &prop_val);
	if (ret)
		return ret;

	if (strcmp(prop_val, "rxaui") == 0)
		gige_mode = false;
	else if (strcmp(prop_val, "xaui") == 0)
		gige_mode = false;
	else if (strcmp(prop_val, "sgmii") == 0)
		gige_mode = true;
	else {
		dev_err(&phydev->dev,
			"Invalid \"cortina,host-mode\" property: \"%s\"\n",
			prop_val);
		return -EINVAL;
	}
	p = devm_kzalloc(&phydev->dev, sizeof(*p), GFP_KERNEL);
	if (!p) {
		dev_err(&phydev->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	p->gige_mode = gige_mode;
	phydev->priv = p;

	ret = sysfs_create_group(&phydev->dev.kobj, &cs4321_attribute_group);
	if (ret) {
		dev_err(&phydev->dev, "Cannot create cs4321 sysfs group\n");
		cs4321_remove(phydev);
		return ret;
	}

	if (IS_ERR(hwmon_device_register(&phydev->dev))) {
		dev_err(&phydev->dev, "Cannot register cs4321 hwmon device\n");
		cs4321_remove(phydev);
		return -ENOMEM;
	}

	ret = sysfs_create_bin_file(&phydev->dev.kobj, &cs4321_sfp_ident_attr);
	if (ret) {
		dev_err(&phydev->dev, "Unable to create sysfs sfp_ident file\n");
		cs4321_remove(phydev);
		return ret;
	}

	ret = sysfs_create_bin_file(&phydev->dev.kobj, &cs4321_sfp_diag_attr);
	if (ret) {
		dev_err(&phydev->dev, "Unable to create sysfs sfp_diag file\n");
		cs4321_remove(phydev);
		return ret;
	}

	return 0;
}

static int cs4321_config_aneg(struct phy_device *phydev)
{
	return -EINVAL;
}

static int cs4321_read_status(struct phy_device *phydev)
{
	struct cs4321_private *p = phydev->priv;
	int v1, v2;

	v1 = cs4321_phy_read(phydev, CS4321_GPIO_GPIO_INPUT_INTS);
	if (v1 < 0)
		return -1;

	if (p->gige_mode) {
		v2 = cs4321_phy_read(phydev, CS4321_GIGEPCS_INT_LINE_PCS1GE_INTSTATUS);
		if (v2 < 0)
			return -1;
		phydev->speed = SPEED_1000;
		phydev->link = !(v1 & 0x0008) && (v2 & 0x0001);
		/* link up if RX_LOS from SFP not asserted and PHY-SFP link up */
	} else {
		v2 = cs4321_phy_read(phydev, CS4321_GPIO_GPIO_INTS);
		if (v2 < 0)
			return -1;
		phydev->speed = SPEED_10000;
		phydev->link = !(v1 & 0x0008) && (v2 & 0x0008);
		/* link up if RX_LOS from SFP not asserted and PHY-SFP link up */
	}
	phydev->duplex = DUPLEX_FULL;

	return 0;
}

static int cs4321_get_module_info(struct phy_device *phydev,
				  struct ethtool_modinfo *modinfo)
{
	u8 data[3];
	int count;

	count = do_sfp_read(phydev, 0, data, 92, 3);
	if (count < 0)
		return count;

	/* Assume SFF-8472 diags registers are available only if ident byte 92
	   reports diag monitoring is implemented, and ident byte 94 reports
	   SFF-8472 compliance. (Some SFPs like FCI 2030LF falsely claim
	   SFF-8472 compliance, so we check both just to be sure.) */
	if ((data[0] & 0x40) && data[2]) {
		modinfo->type = ETH_MODULE_SFF_8472;
		modinfo->eeprom_len = ETH_MODULE_SFF_8472_LEN;
	} else {
		modinfo->type = ETH_MODULE_SFF_8079;
		modinfo->eeprom_len = ETH_MODULE_SFF_8079_LEN;
	}
	
	return 0;
}

static int cs4321_get_module_eeprom(struct phy_device *phydev,
				    struct ethtool_eeprom *ee, u8 *data)
{
	int count;
	int offset = ee->offset;
	int len = ee->len;

	if (offset < 256) {
		int l0 = len;
		if (offset + len > 256)
			l0 = 256 - offset;
		count = do_sfp_read(phydev, 0, data, offset, l0);
		if (count < 0)
			return count;
		offset = 256;
		len -= l0;
		data += l0;
	}
	if (len > 0) {
		count = do_sfp_read(phydev, 1, data, offset - 256, len);
		if (count < 0)
			return count;
	}

	return 0;
}

static struct of_device_id cs4321_match[] = {
	{ .compatible = "cortina,cs4321" },
	{ .compatible = "cortina,cs4318" },
	{},
};

MODULE_DEVICE_TABLE(of, cs4321_match);

static struct phy_driver cs4321_phy_driver = {
	.phy_id = 0xffffffff,
	.phy_id_mask = 0xffffffff,
	.name = "Cortina CS4318/CS4321",
	.config_init = cs4321_config_init,
	.soft_reset = cs4321_soft_reset,
	.remove = cs4321_remove,
	.probe = cs4321_probe,
	.config_aneg = cs4321_config_aneg,
	.read_status = cs4321_read_status,
	.module_info = cs4321_get_module_info,
	.module_eeprom = cs4321_get_module_eeprom,
	.driver = {
		.owner = THIS_MODULE,
		.of_match_table = cs4321_match,
	},
};

static void __init cs4321_add_of_compat(char *match, char *append)
{
	struct device_node *np = NULL;
	while (1) {
		struct property *prop;
		char *val;

		np = of_find_compatible_node(np, NULL, match);
		if (!np)
			break;

		prop = of_find_property(np, "compatible", NULL);
		if (prop) {
			val = kmalloc(prop->length + strlen(append) + 1,
				      GFP_KERNEL);
			strcpy(val, prop->value);
			strcpy(val + prop->length, append);
			prop->value = val;
			prop->length += strlen(append) + 1;
		}
		of_node_put(np);
	}
}

static int __init cs4321_arch_init(void)
{
	cs4321_add_of_compat("cortina,cs4318", "ethernet-phy-id23e5.0002");
	cs4321_add_of_compat("cortina,cs4321", "ethernet-phy-id23e5.0002");
	return 0;
}

arch_initcall(cs4321_arch_init);

static int __init cs4321_drv_init(void)
{
	return phy_driver_register(&cs4321_phy_driver);
}

module_init(cs4321_drv_init);

static void __exit cs4321_drv_exit(void)
{
	phy_driver_unregister(&cs4321_phy_driver);
}

module_exit(cs4321_drv_exit);

static struct mdio_device_id __maybe_unused cs4321_tbl[] = {
	{ 0x23e50002, 0xffff0fff },
	{ }
};

MODULE_DEVICE_TABLE(mdio, cs4321_tbl);

MODULE_DESCRIPTION("Driver for Cortina CS4318/CS4321 PHY");
MODULE_LICENSE("GPL");
