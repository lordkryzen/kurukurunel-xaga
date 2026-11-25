/* SPDX-License-Identifier: GPL-2.0 */
/* Retained xaga charge-pump sysfs ABI from the original mtk_charger.h. */
#ifndef _CHARGE_PUMP_SYSFS_H
#define _CHARGE_PUMP_SYSFS_H
#define CP_SYSFS_FIELD_RO(_name, _prop)	\
{			\
	.attr   = __ATTR(_name, 0444, cp_sysfs_show, cp_sysfs_store),\
	.prop   = _prop,				  \
	.get	= _name##_get,						\
}

enum cp_property {
	CP_PROP_VBUS,
	CP_PROP_IBUS,
	CP_PROP_TDIE,
	CP_PROP_CHIP_OK,
};

#endif
