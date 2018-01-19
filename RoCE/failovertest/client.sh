if [ -f ./filetransfer/test-file ] || [ -f ./sendbyRDMAID/test-combine ];
then
	rm -rf ./filetransfer/test-*
else
	echo "not seeing test-file. skip deletion"
fi

if [ -f ./filetransfer/client ]
then
	mv  ./filetransfer/client ./
else
	echo "not seeing client. skip move"
fi

./client 192.168.7.25 $1 $2

echo "md5 check"
md5sum ./$2
md5sum ./filetransfer/$2
ls -al ./$2
ls -al ./filetransfer/$2
set -x
hexdump ./$2 | head
hexdump ./filetransfer/$2 | head
