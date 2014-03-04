function _roconcomplete_test {
    arg="${COMP_WORDS[COMP_CWORD]}"
    if [[ ${arg} =~ \-\-.* ]]; then
        COMPREPLY=(${COMPREPLY[@]} $(compgen -W "--screen" -- ${arg}))
    else
        _roscomplete_search_dir "( -type f -regex .*\.test$ -o -type f -regex .*\.test$ )"
        if [[ $COMP_CWORD == 1 ]]; then
           COMPREPLY=($(compgen -o plusdirs -f -X "!*.test" -- ${arg}) ${COMPREPLY[@]})
        fi
    fi
}

complete -F "_roconcomplete_test" -o filenames "rocon_test"

