#pragma once
#include <dirent.h>
#include <errno.h>
#include <pwd.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef PAM_MOCK
int pam_get_item(void *p, int i, const void **pwd) { return 0; }
#else
#include <security/pam_appl.h>
#include <security/pam_modules.h>
#endif
#include <json-c/json.h>
#define RED "\x1B[31m"
#define GRN "\x1B[32m"
#define YEL "\x1B[33m"
#define BLU "\x1B[34m"
#define MAG "\x1B[35m"
#define CYN "\x1B[36m"
#define WHT "\x1B[37m"
#define RESET "\x1B[0m"

#define password_folder "/usr/local/src/mirte/mirte-install-scripts/config/pam/"
#define filename password_folder "users.json"
void savePassword(char *, char *);
int checkDirectory();
