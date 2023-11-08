from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import os
import shutil
import time
import asyncio
import traceback
import signal
import sys
import functools
from enum import Enum
import yaml
import subprocess
# watch:
# - pam output
# - nginx output
# no need to do anything with provisioning system, it uses passwd!
watch_pam_output = "/usr/local/src/mirte/mirte-install-scripts/config/pam/users.json"
watch_nginx_output = "/usr/local/src/mirte/mirte-install-scripts/config/web/newpasswd"
# watch_wifi_password = "/home/mirte/.wifi_pwd"


class Update_source(Enum):
    PAM = 1
    NGINX = 2


# update:
# update config/web/password_map.conf && nginx reload
# passwd
# wifi password
wifi_password_file = "/home/mirte/.wifi_pwd"
nginx_output_file = "/usr/local/src/mirte/mirte-install-scripts/config/web/password_map.conf"

# watch files
# - on change:
#   - disable watching
#   - Update files
#   - enable watching
event_loop = asyncio.get_event_loop()


def main():
    # event_loop.add_signal_handler(
    #     signal.SIGINT, functools.partial(asyncio.ensure_future, shutdown())
    # )
    # event_loop.add_signal_handler(
    #     signal.SIGTERM, functools.partial(asyncio.ensure_future, shutdown())
    # )
    observer = Observer()
    event_handler = MyEventHandler()
    observer.schedule(event_handler, watch_nginx_output)
    observer.schedule(event_handler, watch_pam_output)
    observer.start()
    # event_loop.run_until_complete(main_loop())
    # stop_all()


last_copy = time.time()


def get_new_password(source: Update_source) -> str:
    if source == Update_source.PAM:
        # open file and get mirte user
        with open(watch_nginx_output, "r") as file:
            passwords = yaml.safe_load(file)
            if "mirte" in passwords:
                return passwords["mirte"]
            return ""
    if source == Update_source.NGINX:
        with open(watch_nginx_output, "r") as file:
            new_password = file.readline()
            return new_password.strip()
    return ""


def update_nginx(new_password: str) -> None:
    # update map file and reload nginx
    with open(nginx_output_file, "w") as file:
        file.write(f"\"{new_password}\" 1;\n") # "mirte_mirte" 1;
    ret = subprocess.run( # first check config before setting the config.
        f'/bin/bash -c "nginx -t && sudo systemctl reload nginx"',
        capture_output=True,
        shell=True,
    )
    print(ret.stdout.decode(), ret.stderr.decode())


def update_passwd(new_password: str) -> None:
    ret = subprocess.run( # update the real user password by using passwd, this will also update the passwd file
        f'/bin/bash -c "echo -e "{new_password}\n{new_password}" | sudo passwd mirte"',
        capture_output=True,
        shell=True,
    )
    print(ret.stdout.decode(), ret.stderr.decode())



def update_wifi(new_password: str) -> None:
    with open(wifi_password_file, "w") as file:
        file.write(new_password)

def update_passwords(source: Update_source) -> None:
    new_password = get_new_password(source)
    if len(new_password) < 8: # must be at least 8 characters for wifi to accept it
        return

    if source != Update_source.PAM:
        update_passwd(new_password)
    update_nginx(new_password)
    update_wifi(new_password)


# def copy_on_modify(src_path):
#     global last_copy
#     # otherwise it is triggering itself. 1s backoff time
#     if time.time() - last_copy < 1:
#         return
#     last_copy = time.time()
#     if src_path == tmx_config_path:
#         copy(src_path, sd_config_path)
#     else:  # TODO: restart telemetrix node
#         copy(src_path, tmx_config_path)


class MyEventHandler(FileSystemEventHandler):
    def catch_all_handler(self, event):
        if event.is_directory:
            return
        update_src: Update_source = (
            Update_source.PAM
            if event.src_path == watch_pam_output
            else Update_source.NGINX
        )
        update_passwords(update_src)

    def on_moved(self, event):
        self.catch_all_handler(event)

    def on_created(self, event):
        self.catch_all_handler(event)

    def on_deleted(self, event):
        self.catch_all_handler(event)

    def on_modified(self, event):
        print(event)
        self.catch_all_handler(event)


if __name__ == "__main__":
    main()
