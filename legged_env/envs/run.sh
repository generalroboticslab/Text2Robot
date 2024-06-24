#!/bin/bash

# conda activate py38
export LD_LIBRARY_PATH=${CONDA_PREFIX}/lib
experiment_file=exp.sh
task=A1Terrain


function parse_arguments() {
    while [ $# -gt 0 ]; do
        unset OPTIND
        unset OPTARG
        while getopts adpukr options; do
            case $options in
            a)  # profile with austin
                AUSTIN_ARGS="austin --timeout=4s --interval=10us -o ./profile.austin -C"
                ;;
            d)  # debug mode
                export DEBUG=true
                DEBUG="-m debugpy --listen 0.0.0.0:5678 --wait-for-client"
                ;;
            p)  # play mode
                PLAY=true
                ;;
            u)  export PUBLISH_TO_UDP=true ;;
            k)  KEYBOARD=true ;;
            r) DRY_RUN=true ;;
            *) exit ;; #do nothing ;;
            esac
        done
        shift $((OPTIND - 1))
        ARGS="${ARGS} $1 "
        shift
    done
}

function array_to_string {
    # convert arrays of "key=value" to tring, while removing duplicate
    # the latter key_value_pairs will override the formal ones
    # Create an associative array to store unique keys
    declare -A unique_keys
    declare -a orders
    # Create a new array to store the strings without duplicates
    # Loop through each string in the input array
    for string in "${@}"; do
        IFS='=' read -ra parts <<<"$string"
        unique_keys["${parts[0]}"]="${parts[1]}"
        orders+=("${parts[0]}")
    done

    # Remove duplicates while keeping order
    orders=($(echo "${orders[@]}" | tr ' ' '\n' | grep -v '^$' | awk '!seen[$0]++'))

    # associative array to trings
    kv_pairs=()
    for key in "${orders[@]}"; do
        if [[ -z "${unique_keys[$key]}" ]]; then
            kv_pairs+=("$key") # empty value
        else
            kv_pairs+=("$key=${unique_keys[$key]}")
        fi
    done
    # Return the new array without duplicates
    echo "${kv_pairs[@]}"
}


function source_run_commands() {
    # reset ARGS
    BASE_ARGS=()
    TRAIN_ARGS=()
    PLAY_ARGS=()
    KEYBOARD_ARGS=()
    ENTRY_POINT=""

    # check name clashing
    duplicates=$(awk -F'[ (]' '/^function/ {print $2} /^[a-zA-Z_][a-zA-Z0-9_]*\(\)/ {print $1}' $experiment_file | sort | uniq -d)
    [[ -n "$duplicates" ]] && echo -e "${RED}Warning: $experiment_file contains Redefined functions:\n $duplicates${NC}" && exit 1

    source $experiment_file
}

function get_additional_args() {
    $ARG # source setup parameters
    ALL_ARGS=(task=$task ${BASE_ARGS[@]})
    if [ "$KEYBOARD" == "true" ]; then
        ALL_ARGS+=(${KEYBOARD_ARGS[@]})
    fi
    # setting running args depending on mode
    if [ "$PLAY" == "true" ]; then
        ALL_ARGS+=(${PLAY_ARGS[@]})
        # update checkpoint
        [ -e "$checkpoint" ] && ALL_ARGS+=("checkpoint=$checkpoint")
    else # train
        ALL_ARGS+=(${TRAIN_ARGS[@]})
    fi
    ALL_ARGS="$(array_to_string "${ALL_ARGS[@]}")"
}


function run_tasks() {
    for ARG in $ARGS; do
        echo -e "\033[0;32mRunning: [$ARG]\033[0m"
        source_run_commands    
        get_additional_args
        cmd="$AUSTIN_ARGS python -u $DEBUG $ENTRY_POINT $ALL_ARGS"
        # cmd="$AUSTIN_ARGS python -u $DEBUG $ALL_ARGS"
        echo "$cmd"; [[ "$DRY_RUN" != "true" ]] && eval "$cmd"
    done

}
parse_arguments ${@}
run_tasks