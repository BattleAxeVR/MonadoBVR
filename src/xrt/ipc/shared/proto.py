#!/usr/bin/env python3
# Copyright 2020-2023, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0
"""Generate code from a JSON file describing the IPC protocol."""

import argparse

from ipcproto.common import (Proto, write_decl, write_invocation,
                             write_result_handler, write_cpp_header_guard_start,
                             write_cpp_header_guard_end, write_msg_struct,
                             write_reply_struct, write_msg_send)

header = '''// Copyright 2020-2023, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  {brief}.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup ipc{suffix}
 */
'''


def write_send_definition(f, call):
    """Write a ipc_send_CALLNAME_locked function."""
    call.write_send_decl(f)
    f.write("\n{\n")
    f.write("\tIPC_TRACE(ipc_c, \"Sending " + call.name + "\");\n\n")

    write_msg_struct(f, call, '\t')

    write_msg_send(f, 'xrt_result_t ret', indent="\t")

    f.write("\n\treturn ret;\n}\n")


def write_receive_definition(f, call):
    """Write a ipc_receive_CALLNAME_locked function."""
    call.write_receive_decl(f)
    f.write("\n{\n")
    f.write("\tIPC_TRACE(ipc_c, \"Receiving " + call.name + "\");\n\n")

    write_reply_struct(f, call, '\t')

    f.write("\n\t// Await the reply")
    func = 'ipc_receive'
    args = ['&ipc_c->imc', '&_reply', 'sizeof(_reply)']
    write_invocation(f, 'xrt_result_t ret', func, args, indent="\t")
    f.write(";")
    write_result_handler(f, 'ret', None, indent="\t")

    for arg in call.out_args:
        f.write("\t*out_" + arg.name + " = _reply." + arg.name + ";\n")

    f.write("\n\treturn _reply.result;\n}\n")


def write_call_definition(f, call):
    """Write a ipc_call_CALLNAME function."""
    call.write_call_decl(f)
    f.write("\n{\n")

    f.write("\tIPC_TRACE(ipc_c, \"Calling " + call.name + "\");\n\n")

    write_msg_struct(f, call, '\t')
    write_reply_struct(f, call, '\t')

    f.write("""
\t// Other threads must not read/write the fd while we wait for reply
\tos_mutex_lock(&ipc_c->mutex);
""")
    cleanup = "os_mutex_unlock(&ipc_c->mutex);"

    # Prepare initial sending
    write_msg_send(f, 'xrt_result_t ret', indent="\t")
    write_result_handler(f, 'ret', cleanup, indent="\t")

    if call.in_handles:
        f.write("\n\t// Send our handles separately\n")
        f.write("\n\t// Wait for server sync")
        # Must sync with the server so it's expecting the next message.
        write_invocation(
            f,
            'ret',
            'ipc_receive',
            (
                '&ipc_c->imc',
                '&_sync',
                'sizeof(_sync)'
                ),
            indent="\t"
        )
        f.write(';')
        write_result_handler(f, 'ret', cleanup, indent="\t")

        # Must send these in a second message
        # since the server doesn't know how many to expect.
        f.write("\n\t// We need this message data as filler only\n")
        f.write("\tstruct ipc_command_msg _handle_msg = {\n")
        f.write("\t    .cmd = " + str(call.id) + ",\n")
        f.write("\t};\n")
        write_invocation(
            f,
            'ret',
            'ipc_send_handles_' + call.in_handles.stem,
            (
                '&ipc_c->imc',
                "&_handle_msg",
                "sizeof(_handle_msg)",
                call.in_handles.arg_name,
                call.in_handles.count_arg_name
            ),
            indent="\t"
        )
        f.write(';')
        write_result_handler(f, 'ret', cleanup, indent="\t")

    f.write("\n\t// Await the reply")
    func = 'ipc_receive'
    args = ['&ipc_c->imc', '&_reply', 'sizeof(_reply)']
    if call.out_handles:
        func += '_handles_' + call.out_handles.stem
        args.extend(call.out_handles.arg_names)
    write_invocation(f, 'ret', func, args, indent="\t")
    f.write(';')
    write_result_handler(f, 'ret', cleanup, indent="\t")

    for arg in call.out_args:
        f.write("\t*out_" + arg.name + " = _reply." + arg.name + ";\n")
    f.write("\n\t" + cleanup)
    f.write("\n\treturn _reply.result;\n}\n")


def generate_h(file, p):
    """Generate protocol header.

    Defines command enum, utility functions, and command and reply structures.
    """
    f = open(file, "w")
    f.write(header.format(brief='Generated IPC protocol header', suffix=''))
    f.write('''
#pragma once

#include "xrt/xrt_compiler.h"

''')

    write_cpp_header_guard_start(f)
    f.write('''

struct ipc_connection;
''')

    f.write('\ntypedef enum ipc_command')
    f.write('\n{')
    f.write('\n\tIPC_ERR = 0,')
    for call in p.calls:
        f.write("\n\t" + call.id + ",")
    f.write("\n} ipc_command_t;\n")

    f.write('''
struct ipc_command_msg
{
\tenum ipc_command cmd;
};

struct ipc_result_reply
{
\txrt_result_t result;
};

''')

    write_decl(f, return_type='static inline const char *',
        function_name='ipc_cmd_to_str', args=['ipc_command_t id'])
    f.write('\n{')
    f.write('\n\tswitch (id) {')
    f.write('\n\tcase IPC_ERR: return "IPC_ERR";')
    for call in p.calls:
        f.write('\n\tcase ' + call.id + ': return "' + call.id + '";')
    f.write('\n\tdefault: return "IPC_UNKNOWN";')
    f.write('\n\t}\n}\n')

    f.write('#pragma pack (push, 1)')

    for call in p.calls:
        # Should we emit a msg struct.
        if call.needs_msg_struct:
            f.write('\nstruct ipc_' + call.name + '_msg\n')
            f.write('{\n')
            f.write('\tenum ipc_command cmd;\n')
            for arg in call.in_args:
                f.write('\t' + arg.get_struct_field() + ';\n')
            if call.in_handles:
                f.write('\t%s %s;\n' % (call.in_handles.count_arg_type,
                                        call.in_handles.count_arg_name))
            f.write('};\n')
        # Should we emit a reply struct.
        if call.out_args:
            f.write('\nstruct ipc_' + call.name + '_reply\n')
            f.write('{\n')
            f.write('\txrt_result_t result;\n')
            for arg in call.out_args:
                f.write('\t' + arg.get_struct_field() + ';\n')
            f.write('};\n')

    f.write('#pragma pack (pop)\n')

    write_cpp_header_guard_end(f)
    f.close()


def generate_client_c(file, p):
    """Generate IPC client proxy source."""
    f = open(file, "w")
    f.write(header.format(brief='Generated IPC client code', suffix='_client'))
    f.write('''
#include "client/ipc_client.h"
#include "ipc_protocol_generated.h"


\n''')

    # Loop over all of the calls.
    for call in p.calls:
        if call.varlen:
            write_send_definition(f, call)
            write_receive_definition(f, call)
        else:
            write_call_definition(f, call)

    f.close()


def generate_client_h(file, p):
    """Generate IPC client header.

    Contains prototypes for generated IPC proxy call functions.
    """
    f = open(file, "w")
    f.write(header.format(brief='Generated IPC client code', suffix='_client'))
    f.write('''
#pragma once

#include "shared/ipc_protocol.h"
#include "ipc_protocol_generated.h"
#include "client/ipc_client.h"

''')
    write_cpp_header_guard_start(f)
    f.write("\n")

    for call in p.calls:
        if call.varlen:
            call.write_send_decl(f)
            f.write(";\n")
            call.write_receive_decl(f)
        else:
            call.write_call_decl(f)
        f.write(";\n")

    write_cpp_header_guard_end(f)
    f.close()


def generate_server_c(file, p):
    """Generate IPC server stub/dispatch source."""
    f = open(file, "w")
    f.write(header.format(brief='Generated IPC server code', suffix='_server'))
    f.write('''
#include "xrt/xrt_limits.h"

#include "shared/ipc_protocol.h"
#include "shared/ipc_utils.h"

#include "server/ipc_server.h"

#include "ipc_server_generated.h"

''')

    f.write('''
xrt_result_t
ipc_dispatch(volatile struct ipc_client_state *ics, ipc_command_t *ipc_command)
{
\tswitch (*ipc_command) {
''')

    for call in p.calls:
        f.write("\tcase " + call.id + ": {\n")

        f.write("\t\tIPC_TRACE(ics->server, \"Dispatching " + call.name +
                "\");\n\n")

        if call.needs_msg_struct:
            f.write(
                "\t\tstruct ipc_{}_msg *msg = ".format(call.name))
            f.write("(struct ipc_{}_msg *)ipc_command;\n".format(call.name))

        if call.varlen:
            f.write("\t\t// No return arguments")
        elif call.out_args:
            f.write("\t\tstruct ipc_%s_reply reply = {0};\n" % call.name)
        else:
            f.write("\t\tstruct ipc_result_reply reply = {0};\n")

        if call.in_handles:
            # We need to fetch these handles separately
            f.write("\t\tstruct ipc_result_reply _sync = {XRT_SUCCESS};\n")
            f.write("\t\t%s in_%s[XRT_MAX_IPC_HANDLES] = {0};\n" % (
                call.in_handles.typename, call.in_handles.arg_name))
            f.write("\t\tstruct ipc_command_msg _handle_msg = {0};\n")
        if call.out_handles:
            f.write("\t\t%s %s[XRT_MAX_IPC_HANDLES] = {0};\n" % (
                call.out_handles.typename, call.out_handles.arg_name))
            f.write("\t\t%s %s = {0};\n" % (
                call.out_handles.count_arg_type,
                call.out_handles.count_arg_name))
        f.write("\n")

        if call.in_handles:
            # Validate the number of handles.
            f.write("\t\tif (msg->%s > XRT_MAX_IPC_HANDLES) {\n" % (call.in_handles.count_arg_name))
            f.write("\t\t\treturn XRT_ERROR_IPC_FAILURE;\n")
            f.write("\t\t}\n")

            # Let the client know we are ready to receive the handles.
            write_invocation(
                f,
                'xrt_result_t sync_result',
                'ipc_send',
                (
                    "(struct ipc_message_channel *)&ics->imc",
                    "&_sync",
                    "sizeof(_sync)"
                ),
                indent="\t\t"
            )
            f.write(";")
            write_result_handler(f, "sync_result",
                                 indent="\t\t")
            write_invocation(
                f,
                'xrt_result_t receive_handle_result',
                'ipc_receive_handles_' + call.in_handles.stem,
                (
                    "(struct ipc_message_channel *)&ics->imc",
                    "&_handle_msg",
                    "sizeof(_handle_msg)",
                    "in_" + call.in_handles.arg_name,
                    "msg->"+call.in_handles.count_arg_name
                ),
                indent="\t\t"
            )
            f.write(";")
            write_result_handler(f, "receive_handle_result",
                                 indent="\t\t")
            f.write("\t\tif (_handle_msg.cmd != %s) {\n" % str(call.id))
            f.write("\t\t\treturn XRT_ERROR_IPC_FAILURE;\n")
            f.write("\t\t}\n")

        # Write call to ipc_handle_CALLNAME
        args = ["ics"]

        # Always provide in arguments.
        for arg in call.in_args:
            args.append(("&msg->" + arg.name)
                        if arg.is_aggregate
                        else ("msg->" + arg.name))

        # No reply arguments on varlen.
        if not call.varlen:
            args.extend("&reply." + arg.name for arg in call.out_args)

        if call.out_handles:
            args.extend(("XRT_MAX_IPC_HANDLES",
                         call.out_handles.arg_name,
                         "&" + call.out_handles.count_arg_name))

        if call.in_handles:
            args.extend(("&in_%s[0]" % call.in_handles.arg_name,
                         "msg->"+call.in_handles.count_arg_name))

        # Should we put the return in the reply or return it?
        return_target = 'reply.result'
        if call.varlen:
            return_target = 'xrt_result_t xret'

        write_invocation(f, return_target, 'ipc_handle_' +
                         call.name, args, indent="\t\t")
        f.write(";\n")

        # TODO do we check reply.result and
        # error out before replying if it's not success?

        if not call.varlen:
            func = 'ipc_send'
            args = ["(struct ipc_message_channel *)&ics->imc",
                    "&reply",
                    "sizeof(reply)"]
            if call.out_handles:
                func += '_handles_' + call.out_handles.stem
                args.extend(call.out_handles.arg_names)
            write_invocation(f, 'xrt_result_t xret', func, args, indent="\t\t")
            f.write(";")

        f.write("\n\t\treturn xret;\n")
        f.write("\t}\n")
    f.write('''\tdefault:
\t\tU_LOG_E("UNHANDLED IPC MESSAGE! %d", *ipc_command);
\t\treturn XRT_ERROR_IPC_FAILURE;
\t}
}

''')

    f.write('''
size_t
ipc_command_size(const enum ipc_command cmd)
{
\tswitch (cmd) {
''')

    for call in p.calls:
        if call.needs_msg_struct:
            f.write("\tcase " + call.id + ": return sizeof(struct ipc_{}_msg);\n".format(call.name))
        else:
            f.write("\tcase " + call.id + ": return sizeof(enum ipc_command);\n")

    f.write('''\tdefault:
\t\tU_LOG_E("UNHANDLED IPC COMMAND! %d", cmd);
\t\treturn 0;
\t}
}

''')

    f.close()


def generate_server_header(file, p):
    """Generate IPC server header.

    Declares handler prototypes to implement,
    as well as the prototype for the generated dispatch function.
    """
    f = open(file, "w")
    f.write(header.format(brief='Generated IPC server code', suffix='_server'))
    f.write('''
#pragma once

#include "shared/ipc_protocol.h"
#include "ipc_protocol_generated.h"
#include "server/ipc_server.h"


''')

    write_cpp_header_guard_start(f)
    f.write("\n")

    # Those decls are constant, but we must write them here
    # because they depends on a generated enum.
    write_decl(
        f,
        "xrt_result_t",
        "ipc_dispatch",
        [
            "volatile struct ipc_client_state *ics",
            "ipc_command_t *ipc_command"
        ]
    )
    f.write(";\n")

    write_decl(
        f,
        "size_t",
        "ipc_command_size",
        [
            "const enum ipc_command cmd"
        ]
    )
    f.write(";\n")

    for call in p.calls:
        call.write_handler_decl(f)
        f.write(";\n")

    write_cpp_header_guard_end(f)
    f.close()


def main():
    """Handle command line and generate a file."""
    parser = argparse.ArgumentParser(description='Protocol generator.')
    parser.add_argument(
        'proto', help='Protocol file to use')
    parser.add_argument(
        'output', type=str, nargs='+',
        help='Output file, uses the name to choose output type')
    args = parser.parse_args()

    p = Proto.load_and_parse(args.proto)

    for output in args.output:
        if output.endswith("ipc_protocol_generated.h"):
            generate_h(output, p)
        if output.endswith("ipc_client_generated.c"):
            generate_client_c(output, p)
        if output.endswith("ipc_client_generated.h"):
            generate_client_h(output, p)
        if output.endswith("ipc_server_generated.c"):
            generate_server_c(output, p)
        if output.endswith("ipc_server_generated.h"):
            generate_server_header(output, p)


if __name__ == "__main__":
    main()
