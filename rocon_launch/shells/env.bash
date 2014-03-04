function _roconcomplete_launch {
    arg="${COMP_WORDS[COMP_CWORD]}"
    COMPREPLY=()
    if [[ ${arg} =~ \-\-.* ]]; then
        COMPREPLY=(${COMPREPLY[@]} $(compgen -W "--konsole --gnome --screen --no-terminal" -- ${arg}))
    else
        _roscomplete_search_dir "( -type f -regex .*\.concert$ -o -type f -regex .*\.concert$ )"
        if [[ $COMP_CWORD == 1 ]]; then
           COMPREPLY=($(compgen -o plusdirs -f -X "!*.concert" -- ${arg}) ${COMPREPLY[@]})
        fi
    fi
}

complete -F "_roconcomplete_launch" -o filenames "rocon_launch"

