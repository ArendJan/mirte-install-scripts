#define PAM_SM_PASSWORD 1
#include "mirte_pam.h"
PAM_EXTERN int pam_sm_chauthtok(pam_handle_t *pamh, int flags, int argc,
                                const char **argv) {
  if (flags & PAM_PRELIM_CHECK) {
    return PAM_SUCCESS;
  }
  char *pwd;
  char *user;
  int r = pam_get_item(pamh, PAM_AUTHTOK, (const void **)&pwd);
  pam_get_item(pamh, PAM_USER, (const void **)&user);
  savePassword(user, pwd);
  return PAM_SUCCESS;
}

void savePassword(char *username, char *passwd) {
  // Don't show anything when it is not the mirte user
  if (strcmp(username, mirte_username) != 0) {
    return;
  }
  printf(GRN
         "Mirte:\t" RESET "The new password for \"%s\" is \"%s\".\n" GRN
         "Mirte:\t" RESET "The password will be updated for the webpages.\n" GRN
         "Mirte:\t" RESET "The Wi-Fi password will be updated at next boot!\n",
         username, passwd);
  if (strlen(passwd) < 8) {
    printf(RED "Mirte:\t" RESET "Password is too short, not storing "
               "password for Wi-Fi.\n");
    return;
  }
  if (!checkDirectory()) {
    printf(RED "Mirte:\t" RESET
               "Mirte home directory does not exist, not storing "
               "password for Wi-Fi.\n");
    return;
  }
  FILE *file = fopen(wifi_filename, "w");
  fprintf(file, "%s", passwd);
  fclose(file);
}

int checkDirectory() {
  DIR *dir = opendir(wifi_password_folder);
  if (dir) {
    /* Directory exists. */
    closedir(dir);
    return 1;
  } else // ENOENT or any other value
  {
    return 0;
  }
}