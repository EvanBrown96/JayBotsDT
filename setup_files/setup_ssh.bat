ssh %1 mkdir -p .ssh
cat %USERPROFILE%\.ssh\id_rsa.pub | ssh %1 "cat >> ~/.ssh/authorized_keys"
