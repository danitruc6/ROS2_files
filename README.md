# ROS2_files
Repo with my setup and ros2 files


Usuful Ubunto packages 
eza:
sudo apt update
sudo apt install -y gpg

sudo mkdir -p /etc/apt/keyrings
wget -qO- https://raw.githubusercontent.com/eza-community/eza/main/deb.asc | sudo gpg --dearmor -o /etc/apt/keyrings/gierens.gpg
echo "deb [signed-by=/etc/apt/keyrings/gierens.gpg] http://deb.gierens.de stable main" | sudo tee /etc/apt/sources.list.d/gierens.list
sudo chmod 644 /etc/apt/keyrings/gierens.gpg /etc/apt/sources.list.d/gierens.list
sudo apt update
sudo apt install -y eza

zoxyde
curl -sSfL https://raw.githubusercontent.com/ajeetdsouza/zoxide/main/install.sh | sh

Bash
Add this to the end of your config file (usually ~/.bashrc):

eval "$(zoxide init bash)"

Zsh
Add this to the end of your config file (usually ~/.zshrc):

eval "$(zoxide init zsh)"

fzf
sudo apt install fzf

fd
sudo apt install fd-find

check  and create .local/bin directory
Make sure that $HOME/.local/bin is in your $PATH, by adding this line into .bashrc
export PATH="$HOME/.local/bin:$PATH"

ln -s $(which fdfind) ~/.local/bin/fd


btop
sudo apt install btop

npm and nodejs


autocompletion
sudo apt install bash-completion


lines added to .zshrc:
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
export PATH="$HOME/.local/bin:$PATH"

source /opt/ros/humble/setup.zsh
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
# source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source ~/ros2_ws/install/setup.zsh

eval "$(zoxide init zsh)"
alias cd="z"
alias ls="eza --color=always --long --no-filesize --icons=always --no-time --no-user --no-permissions"
