#define PAM_SM_PASSWORD 1
#include "mirte_pam.h"
PAM_EXTERN int pam_sm_chauthtok(pam_handle_t *pamh, int flags, int argc,
                                const char **argv) {

  if (flags == PAM_PRELIM_CHECK) {
    return PAM_SUCCESS;
  }
  char *pwd;
  char *user;
  int r = pam_get_item(pamh, PAM_AUTHTOK, (const void **)&pwd);
  pam_get_item(pamh, PAM_USER, (const void **)&user);
  printf(GRN
         "Mirte:\t" RESET "The new password for \"%s\" is \"%s\".\n" GRN
         "Mirte:\t" RESET "The password will be updated for the webpages.\n" GRN
         "Mirte:\t" RESET "The Wi-Fi password will be updated at next boot!\n",
         user, pwd);
  savePassword(user, pwd);
  return PAM_SUCCESS;
}

void savePassword(char *username, char *passwd) {
  if (!checkDirectory()) {
    printf(GRN "Mirte:\t" RESET "Mirte directory does not exist, not storing "
               "password for webpages and Wi-Fi.\n");
    return;
  }
  json_object *root = json_object_from_file(filename);
  if (!root) {
    // file did not exist or some other error, just remake it.
    root = json_object_new_object();
  }

  json_object *user = json_object_object_get(root, username);
  if (user) {
    json_object_set_string(user, passwd);
  } else {
    // user does not exist yet, so add it to the file.
    json_object_object_add(root, username, json_object_new_string(passwd));
  }
  json_object_to_file_ext(filename, root, JSON_C_TO_STRING_PRETTY);
  json_object_put(root);
}

int checkDirectory() {
  DIR *dir = opendir(password_folder);
  if (dir) {
    /* Directory exists. */
    closedir(dir);
    return 1;
  } else // ENOENT or any other value
  {
    return 0;
  }
}