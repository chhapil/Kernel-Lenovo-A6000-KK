#!/bin/bash

# 2.) clean the sources                                                     #
echo "Cleaning kernel now";
bash ./clean-junk.sh; 
yes | bash ./clean-kernel.sh;   

# 1.) load the ".config"                                                    #
echo "Loading config file";
bash ./load_config.sh;                                                      #
                                                                            #
# 3.) now you can build my kernel                                           #
bash ./build_kernel.sh                                                      #

#                                                                           #
# Have fun and update me if something nice can be added to my source.       #
#############################################################################
