# password module

This system will show some info to the user when changing passwords and stores the new password in `/home/mirte/.wifi_pwd`. The warnings are hooked in to `passwd` by using pam modules. It will only print and store when the mirte user password is changed, not when changing the root password.

Output:

![passwd with mirte pam](image.png)

# Build
Building PAM modules requires `libpam0g-dev`:

```sh
sudo apt-get install libpam0g-dev
```

Building the Mirte PAM system:
```sh
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
make install # requires sudo privileges
```

This will build the Mirte pam modules(warning and storepassword), installs them to `/lib/security/limbirte_pam_xxx.so` and add the required lines to `/etc/pam.d/passwd` and `/etc/pam.d/common-password`.

### passwd:
The resulting `/etc/pam.d/passwd` should look like:
```
#
# The PAM configuration file for the Shadow `passwd' service
#

password required /lib/security/libmirte_pam_warn.so
@include common-password
```

### common-password:
The resulting `/etc/pam.d/common-password` should look like:

```
...
password required /lib/security/libmirte_pam_storepassword.so
```

The `warn` module is started before the actual passwd system, so the output will be before the passwd output. The storepassword is at the end to catch the plaintext password.