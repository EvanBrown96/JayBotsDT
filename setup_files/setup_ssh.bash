ssh $0 mkdir -p .ssh
cat ~/.ssh/id_rsa.pub | ssh $0 'cat >> ~/.ssh/authorized_keys'
