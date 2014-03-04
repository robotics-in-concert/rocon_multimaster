# No idea if this actually works yet

function _roconcomplete_launchfile {
    _roconcomplete_search_dir "( -type f -regex .*\.concert$ -o -type f -regex .*\.test$ )"
}

compctl -/g '*.(concert|test)' -x 'p[1]' -K "_roconcomplete" -tx - 'p[2]' -K _roconcomplete_launchfile -- + -x 'S[--]' -k "(--konsole --gnome --screen)" -- "rocon_launch"
