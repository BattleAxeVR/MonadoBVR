// Copyright 2019-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Prints a list of found devices and tests opening some of them.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Korcan Hussein <korcan.hussein@collabora.com>
 */

#include "xrt/xrt_space.h"
#include "xrt/xrt_system.h"
#include "xrt/xrt_prober.h"
#include "xrt/xrt_instance.h"

#include "cli_common.h"

#include <string.h>
#include <stdio.h>


static int
do_exit(struct xrt_instance **xi_ptr, int ret)
{
	xrt_instance_destroy(xi_ptr);

	printf(" :: Exiting '%i'\n", ret);

	return ret;
}

#define NUM_XDEVS 32

int
cli_cmd_test(int argc, const char **argv)
{
	struct xrt_instance *xi = NULL;
	xrt_result_t xret = XRT_SUCCESS;
	int ret = 0;

	// Initialize the prober.
	printf(" :: Creating instance!\n");

	ret = xrt_instance_create(NULL, &xi);
	if (ret != 0) {
		return do_exit(&xi, 0);
	}
	struct xrt_prober *xp = NULL;

	xret = xrt_instance_get_prober(xi, &xp);
	if (xret != XRT_SUCCESS) {
		do_exit(&xi, ret);
	}
	if (xp != NULL) {
		// This instance provides an xrt_prober so we can dump some
		// internal info.

		// Need to prime the prober with devices before dumping and
		// listing.
		printf(" :: Probing!\n");

		xret = xrt_prober_probe(xp);
		if (xret != XRT_SUCCESS) {
			return do_exit(&xi, -1);
		}

		// So the user can see what we found.
		printf(" :: Dumping!\n");

		ret = xrt_prober_dump(xp, true);
		if (ret != 0) {
			do_exit(&xi, ret);
		}
	}

	// Regardless of whether xrt_prober is used, we can find and select
	// (multiple) devices.
	printf(" :: Creating system devices!\n");

	struct xrt_system *xsys = NULL;
	struct xrt_system_devices *xsysd = NULL;
	struct xrt_space_overseer *xso = NULL;
	xret = xrt_instance_create_system( //
	    xi,                            // Instance
	    &xsys,                         // System
	    &xsysd,                        // System devices.
	    &xso,                          // Space Overseer.
	    NULL);                         // System compositor.
	if (xret != XRT_SUCCESS) {
		printf("\tCall to xrt_instance_create_system failed! '%i'\n", xret);
		return do_exit(&xi, -1);
	}
	if (xsysd == NULL) {
		printf("\tNo xrt_system_devices returned!\n");
		return do_exit(&xi, -1);
	}
	if (xsysd->xdevs[0] == NULL) {
		printf("\tNo HMD found! :(\n");
		return do_exit(&xi, -1);
	}

	printf(" :: Listing created devices!\n");

	for (uint32_t i = 0; i < XRT_SYSTEM_MAX_DEVICES; i++) {
		if (xsysd->xdevs[i] == NULL) {
			continue;
		}

		printf("\t%2u: %s\n", i, xsysd->xdevs[i]->str);
	}

	struct xrt_system_roles roles = XRT_SYSTEM_ROLES_INIT;
	xrt_system_devices_get_roles(xsysd, &roles);

	printf(" :: Listing role assignments!\n");

#define PRINT_ROLE(ROLE, PAD)                                                                                          \
	do {                                                                                                           \
		if (xsysd->static_roles.ROLE == NULL) {                                                                \
			printf("\t" #ROLE ": " PAD "<none>\n");                                                        \
		} else {                                                                                               \
			printf("\t" #ROLE ": " PAD "%s\n", xsysd->static_roles.ROLE->str);                             \
		}                                                                                                      \
	} while (false)

#define PRINT_DYNR(ROLE, PAD)                                                                                          \
	do {                                                                                                           \
		if (roles.ROLE < 0) {                                                                                  \
			printf("\t" #ROLE ": " PAD "<none>\n");                                                        \
		} else {                                                                                               \
			printf("\t" #ROLE ": " PAD "%s\n", xsysd->xdevs[roles.ROLE]->str);                             \
		}                                                                                                      \
	} while (false)

	PRINT_ROLE(head, "               ");
	PRINT_ROLE(eyes, "               ");
	PRINT_ROLE(face, "               ");
	PRINT_ROLE(body, "               ");
	PRINT_DYNR(left, "               ");
	PRINT_DYNR(right, "              ");
	PRINT_DYNR(gamepad, "            ");
	PRINT_ROLE(hand_tracking.left, " ");
	PRINT_ROLE(hand_tracking.right, "");


	// End of program
	printf(" :: All ok, shutting down.\n");

	xrt_space_overseer_destroy(&xso);
	xrt_system_devices_destroy(&xsysd);
	xrt_system_destroy(&xsys);

	// Finally done
	return do_exit(&xi, 0);
}
