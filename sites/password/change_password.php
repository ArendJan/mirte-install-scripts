<?php
// get POST parameters
$fields = json_decode(file_get_contents('php://input'), true);
if (!is_array($fields)) {
    $msg = "Invalid JSON request body.";
    report();
    exit;
}
$old_password = (string)($fields['old_password'] ?? '');
$new_password = (string)($fields['new_password'] ?? '');

$ok = false;
$msg = "";
function report() {
    global $ok, $msg;
    header('Content-Type: application/json');
    echo json_encode(array("ok" => $ok, "msg" => $msg));
}

// validate input
if (empty($old_password) || empty($new_password)) {
    $msg = "Please fill in all fields.";
    report();
    exit;
}

if (strlen($new_password) < 8) {
    $msg = "New password must be at least 8 characters long.";
    report();
    exit;
}

// only allow letters, numbers, - and _ in the new password, this makes it safe to use in eval.
 if (!preg_match('/^[a-zA-Z0-9_-]+$/', $new_password)) {
     $msg = "New password can only contain letters, numbers, - and _ .";
     report();
     exit;
 }

// just make sure old password is also safe to use in eval, although it will be passed to su and sudo, so it can contain any character.
$old_password = escapeshellarg($old_password);

$new_password = escapeshellarg($new_password); // works with the echo and chpasswd.

// set new password for user mirte
// chpasswd gets mirte:<new_pw> from stdin
// su gets the old password from stdin to check that the old password is correct
// sudo (chpasswd) doesnt need a password as mirte (from su) is in the sudoers file with NOPASSWD

// su will return after a few seconds if incorrect, otherwise it's 'instantaneous' and the old password was correct.
// return code 0 means success, other means failure (incorrect old password)
exec("echo $old_password | su -c \"echo mirte:$new_password | sudo chpasswd \" mirte", $out, $return_var);
if ($return_var === 0) {
    $msg = "Password changed successfully.";
    $ok = true;
} else {
    $msg = "Failed to change password: " . join("\n", $out) . " (return code: $return_var)";
    $ok = false;
}
report();

?>