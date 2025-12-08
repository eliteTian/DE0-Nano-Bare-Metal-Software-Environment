fatload mmc 0:1 ${loadaddr} bm.scr

source ${loadaddr}

screen: CTRL+A   SHIFT+H

sed -i 's/[\x00-\x1F\x7F]//g' screenlog.0 

