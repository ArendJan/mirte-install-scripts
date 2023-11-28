#define PAM_SM_PASSWORD 1
#include "mirte_pam.h"
/*
    This module will warn users when changing their password that it is stored
   in plaintext Compile with gcc -fPIC -fno-stack-protector -c warn.c && ld -x
   --shared -o /lib/security/warn.so warn.o and add password required
   /lib/security/warn.so before the "@include common-password" line in
   /etc/pam.d/passwd to let it run before the passwd system.
*/
PAM_EXTERN int pam_sm_chauthtok(pam_handle_t *pamh, int flags, int argc,
                                const char **argv) {
  if (flags == PAM_PRELIM_CHECK) {
    printf(GRN
           "Mirte:\t" RESET "Your password will be stored in plaintext, so "
           "don't make it your super secret password!\n" GRN "Mirte:\t" RESET
           "The web-login and the Mirte-hotspot will get the same password.\n");
    return PAM_SUCCESS;
  }

  return PAM_SUCCESS;
}