# No idea if this actually works
complete rocon_launch \
   'p/1/`rospack list-names && find . -maxdepth 3 -regex ".*[xl][am][ul]n*c*h*" -printf "%P\n" `/' \
   'p@2@`rospack find $:1 | xargs -I 1 find 1 -maxdepth 3 -regex ".*[xl][am][ul]n*c*h*" -type f -printf "%f\n"`@'

