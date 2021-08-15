
#include "shell/shell.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define SHELL_RX_BUFFER_SIZE (256)
#define SHELL_MAX_ARGS (16)
#define SHELL_PROMPT "shell> "

// By default, enable shell RX feedback
#ifndef SHELL_ENABLE_RX_FEEDBACK
	#define ENABLE_RX_FEEDBACK
#endif

#define SHELL_FOR_EACH_COMMAND(command) \
  for (const sShellCommand *command = g_shell_commands; \
    command < &g_shell_commands[g_num_shell_commands]; \
    ++command)

static struct ShellContext {
  int (*send_printf)(const char* format, ...);
  size_t rx_size;
  char rx_buffer[SHELL_RX_BUFFER_SIZE];
} s_shell;

static bool prv_booted(void) {
  return s_shell.send_printf != NULL;
}

#define SHELL_PRINTF(f_, ...) if (prv_booted()) s_shell.send_printf((f_), ##__VA_ARGS__)

static char prv_last_char(void) {
  return s_shell.rx_buffer[s_shell.rx_size - 1];
}

static bool prv_is_rx_buffer_full(void) {
  return s_shell.rx_size >= SHELL_RX_BUFFER_SIZE;
}

static void prv_reset_rx_buffer(void) {
  memset(s_shell.rx_buffer, 0, sizeof(s_shell.rx_buffer));
  s_shell.rx_size = 0;
}

static void prv_send_prompt(void) {
  SHELL_PRINTF(SHELL_PROMPT);
}

static const sShellCommand *prv_find_command(const char *name) {
  SHELL_FOR_EACH_COMMAND(command) {
    if (strcmp(command->command, name) == 0) {
      return command;
    }
  }
  return NULL;
}

static void prv_process(void) {
  if (prv_last_char() != '\n' && !prv_is_rx_buffer_full()) {
    return;
  }

  char *argv[SHELL_MAX_ARGS] = {0};
  int argc = 0;

  char *next_arg = NULL;
  for (size_t i = 0; i < s_shell.rx_size && argc < SHELL_MAX_ARGS; ++i) {
    char *const c = &s_shell.rx_buffer[i];
    if (*c == ' ' || *c == '\n' || i == s_shell.rx_size - 1) {
      *c = '\0';
      if (next_arg) {
        argv[argc++] = next_arg;
        next_arg = NULL;
      }
    } else if (!next_arg) {
      next_arg = c;
    }
  }

  if (s_shell.rx_size == SHELL_RX_BUFFER_SIZE) {
    SHELL_PRINTF("\n");
  }

  if (argc >= 1) {
    const sShellCommand *command = prv_find_command(argv[0]);
    if (!command) {
      SHELL_PRINTF("Unknown command: %s\nType 'help' to list all commands\n", argv[0]);
    } else {
      command->handler(argc, argv);
    }
  }
  prv_reset_rx_buffer();
  prv_send_prompt();
}

void shell_boot(const sShellImpl *impl) {
  s_shell.send_printf = impl->send_printf;
  prv_reset_rx_buffer();
  SHELL_PRINTF("\n" SHELL_PROMPT);
}

void shell_receive_char(char c) {
  if (c == '\r' || prv_is_rx_buffer_full() || !prv_booted()) {
    return;
  }
  #ifdef ENABLE_RX_FEEDBACK
    SHELL_PRINTF("%c", c);
  #endif

  if (c == '\b') {
    s_shell.rx_buffer[--s_shell.rx_size] = '\0';
    return;
  }

  s_shell.rx_buffer[s_shell.rx_size++] = c;

  prv_process();
}

int shell_help_handler(int argc, char *argv[]) {
  SHELL_FOR_EACH_COMMAND(command) {
    SHELL_PRINTF("%s: %s\n", command->command, command->help);
  }
  return 0;
}
