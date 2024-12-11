# preshared wifi setup

- set the seed in compile and setup
- set ssid in compile and setup
- add all macs and passwords in the csv (no spaces after comma)
- run compile_wifi.sh to create the <ssid> folder with the hashed and encrypted files
- run `shc setup_wifi.sh` to compile it to keep the salt 'secure'
- Copy the <ssid> folder and setup_wifi.sh to target machine
- run setup_wifi.sh.x on target machine when you want to connect



The passwords are hashed, then encrypted with their mac and a salt. On trying to connect, the script tries to decrypt all the encrypted files. One should be able to be decrypted, that psk is then used to connect to the network. The setup script is compiled to (kinda) protect the salt.