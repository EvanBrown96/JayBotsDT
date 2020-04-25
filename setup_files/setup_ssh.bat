ssh -oHostKeyAlgorithms=ssh-rsa %1 mkdir -p .ssh
type %USERPROFILE%\.ssh\id_rsa.pub | ssh %1 "cat >> ~/.ssh/authorized_keys"
