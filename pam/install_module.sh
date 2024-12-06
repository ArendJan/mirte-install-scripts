#!/bin/bash

#check if already added to these settings and put it in the correct place if not yet
echo "installing mirte pam modules when needed"
if ! grep -q libmirte_pam_warn "/etc/pam.d/passwd"; then
	sed -i '/@include common-password/i \
password required /lib/security/libmirte_pam_warn.so' /etc/pam.d/passwd
	echo "added warning module to /etc/pam.d/passwd"
fi

if ! grep -q libmirte_pam_storepassword "/etc/pam.d/common-password"; then
	printf "\npassword required /lib/security/libmirte_pam_storepassword.so\n" >>/etc/pam.d/common-password
	echo "added password storing to /etc/pam.d/common-password"
fi
