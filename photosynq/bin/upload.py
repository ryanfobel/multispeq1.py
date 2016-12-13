from arduino_helpers.upload import upload, parse_args
from .. import get_firmwares


if __name__ == '__main__':
    args = parse_args()

    print upload(args.board_name, lambda b: get_firmwares()[b][0], args.port,
                 args.arduino_install_home, verify=not args.skip_verify)
