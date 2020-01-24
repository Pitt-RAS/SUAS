# SUAS
Code to Power Technology Demonstrator for SUAS

# Brief Note on the .devcontainer
A .env file in .devcontainer is required for used in the for the development container. 
Most critically, it sets the $SSH_AUTH_SOCK for ssh-agent forwarding. If you're a Mac/WSL 2
user, you may wish to set this ```/run/host-services/ssh-auth.sock```. This should allow
the ssh-agent to properly forward to the dev container. If you're on Linux, no intervention
on your part should be required.