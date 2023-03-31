import os
import argparse

def parse_config():
    parser = argparse.ArgumentParser(description='arg parser')
    parser.add_argument('--input', type=str, default=None,
                        help='specify the system zip file')
    parser.add_argument('--signature', type=str, default=None,
                        help='signature zip file')
    parser.add_argument('--out', type=str, default=None,
                        help='specify the output file')

    args = parser.parse_args()
    return args

def get_current_version():
    if not os.path.exists('VERSION'):
        return 'Dev'
    else:
        f = open('VERSION', 'r')
        return f.read().replace("\n","")

def main():
    args = parse_config()

    version = get_current_version()
    print("version: ", version)
    version = bytes(version, encoding = "utf8")
    version_len = len(version)
    version_len = int(version_len).to_bytes(length=4, byteorder='big', signed=False)

    # get lastest tag name
    process = os.popen('git describe --tags `git rev-list --tags --max-count=1`')
    tag_name = process.read().replace("\n","")

    # get tag summery
    process = os.popen('git show {} --summary'.format(tag_name))
    release_note = process.read()
    # print("release_note: ", release_note)
    release_note = bytes(release_note, encoding = "utf8", errors='ignore')
    release_note_len = len(release_note)
    release_note_len = int(release_note_len).to_bytes(length=4, byteorder='big', signed=False)

    fi = open(args.input, 'rb')
    system_bytes = fi.read()

    fs = open(args.signature, 'rb')
    signature_bytes = fs.read()
    print('signature bytes:', len(signature_bytes), signature_bytes)

    fo = open(args.out, 'wb')
    magic_number = bytes('TSARI', encoding = "utf8")
    fo.write(magic_number)

    fo.write(version_len)
    fo.write(version)

    fo.write(release_note_len)
    fo.write(release_note)

    fo.seek(4096)
    fo.write(signature_bytes)
    fo.write(system_bytes)

if __name__ == '__main__':
    main()