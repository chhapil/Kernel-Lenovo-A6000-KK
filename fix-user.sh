#!/bin/bash
#Title=Make owned by current user
#Title[se]=Gör nuvarande användare till ägare
 
# Make owned by current user - Makes the selected files owned by the current user with group 
# being the user's primary group (the first in the output from the "groups" command)

# Installation:
# Put this script into the Nautilus script dir (~/.gnome2/nautilus-scripts) and make it executable.
#
# Usage:
# Right-click on files in Nautilus and choose Scripts -> Make owned by current user
#
# Notes:
# This operates non-recursively by default, so it will only operate on the specific folder and files that 
# you have selected. If you want recursive behavior - uncomment the "#RECURSIVE=-R;" line further down.
# It is left out be default since the risk of messing up ownerships is pretty big when something 
# like this gets so easy. 
#
# Acknowledements and version history:
# v20080131 - Fredrik Wollsén
#
# License GPL v3
#
# Feel free to provide feedback on this script here:
# http://ubuntuforums.org/showthread.php?t=683945
#
# Suggestions for improvements:
#  - Show a zenity dialogue box to dynamically decide whether or not the command 
#    should be run recursively or not.
#  - Show a zenity progress bar for the execution of the command.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
# IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
# IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

USER=`whoami`;
GROUP=`groups | sed -r 's/ .*//g'`;
#RECURSIVE=-R; # Uncomment this to make the ownerships be implemented resursively

# default to a group name identical to the username if a group is not found (is this case even possible? this if-statement could be totally useless - but: better safe than sorry...)
if [ "$GROUP" == "" ] ; then
GROUP=$USER;
fi

find . -type d \! -user $USER -exec sudo chown -v $RECURSIVE $USER:$GROUP {} \;
find . -type f \! -user $USER -exec sudo chown -v $RECURSIVE $USER:$GROUP {} \;
exit 0;