/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * This header provides common constants for PCI bindings.
 */

#ifndef _DT_BINDINGS_PCI_PCI_H
#define _DT_BINDINGS_PCI_PCI_H

/* Encode a vendor and device ID into a single cell */
#define PCI_VENDEV(v, d)	(((v) << 16) | (d))

#define true		1
#define false		0

#define EP_NO_DEVICE		0
#define EP_BCM_WIFI		1
#define EP_SAMSUNG_S359		2
#define EP_QC_MODEM		3
#define EP_SAMSUNG_MODEM	4
#define EP_QC_WIFI		5
#define EP_SAMSUNG_WIFI		6

/*
 * CAUTION - It SHOULD fit Target Link Speed Encoding
 * in Link Control2 Register(offset 0xA0)
 */
#define LINK_SPEED_GEN1		1
#define LINK_SPEED_GEN2		2
#define LINK_SPEED_GEN3		3

#endif /* _DT_BINDINGS_PCI_PCI_H */
