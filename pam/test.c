#define PAM_MOCK
#include "mirte_pam.h"

int main() {
  // will just update the file
  printf("Not mirte user test:\n");
  savePassword("asdf", "ww");
  printf("Mirte user test:\n");
  savePassword("mirte", "ww");
}