import os
import subprocess
import provisioning_module

auth_keys_path = "/home/mirte/.ssh/authorized_keys"


class SSHConfig(provisioning_module.ProvisionModule):
    needs_mount = True

    def start(self, mount_point, loop):
        config_file = f"{mount_point}/authorized_keys"
        if not os.path.isfile(config_file):
            print("No authorized keys configuration")
            return
        existing_keys = []
        # if not exists, create the .ssh folder and the authorized_keys file
        if not os.path.isdir(os.path.dirname(auth_keys_path)):
            os.makedirs(os.path.dirname(auth_keys_path), mode=0o700)
        if not os.path.isfile(auth_keys_path):
            open(auth_keys_path, "a").close()
            os.chmod(auth_keys_path, 0o600)
            # give ownership to the user mirte, so that the user can add keys later on
            subprocess.run(
                f"chown -R mirte:mirte {os.path.dirname(auth_keys_path)}",
                shell=True,
                check=True,
            )
        with open(config_file, "r") as file:
            new_keys = file.readlines()
        if os.path.isfile(auth_keys_path):
            with open(auth_keys_path) as file:
                existing_keys = file.readlines()

        new_keys = list(filter(lambda key: not key in existing_keys, new_keys))
        subprocess.run(
            f"chown -R mirte:mirte {os.path.dirname(auth_keys_path)}",
            shell=True,
            check=True,
        )
        os.chmod(auth_keys_path, 0o600)
        if len(new_keys) == 0:
            print("No new SSH keys to add")
            return
        print("adding:", new_keys)
        with open(auth_keys_path, "a") as file:
            file.writelines(new_keys)
        # fix permissions just in case
        # sudo chown -R $USER:$USER ~/.ssh
        # chmod -R 700 ~/.ssh

        print("SSH authorized keys updated successfully.")

    async def stop(self):
        print("stop ssh")
