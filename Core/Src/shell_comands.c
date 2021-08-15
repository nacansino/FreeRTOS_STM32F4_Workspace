/**
 * @file shell_comands.c
 * @author Niel Cansino (nielcansino@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-08-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <main.h>
#include <shell/shell.h>

#include <stddef.h>
#include <stdio.h>

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

// A simple Hello World command which prints "Hello World!"
int cli_cmd_hello(int argc, char *argv[]) {
  Shell_Printf("Hello World!");
  return 0;
}

static const sShellCommand s_shell_commands[] = {
  {"hello", cli_cmd_hello, "Say hello"},
  {"help", shell_help_handler, "Lists all commands"},
};

const sShellCommand *const g_shell_commands = s_shell_commands;
const size_t g_num_shell_commands = ARRAY_SIZE(s_shell_commands);
