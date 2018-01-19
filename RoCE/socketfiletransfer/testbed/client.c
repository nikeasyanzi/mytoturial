#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#define PORT 1234

int main(int argc, char const *argv[])
{
	int sockfd = 0;
	FILE * fp;
	int valread;
	struct sockaddr_in serv_addr;
	char buffer[1024] = {0};
	int numbytes;

	if(argc!=3){

		printf("please use ./client server_ip filename\n");
		return -1;
	}
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		printf("\n Socket creation error \n");
		return -1;
	}

	bzero( &serv_addr, sizeof(serv_addr) );
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = inet_addr(argv[1]);
	serv_addr.sin_port = htons(PORT);

	// Convert IPv4 and IPv6 addresses from text to binary form
	/*if(inet_pton(AF_INET, argv[1], &serv_addr.sin_addr)<=0)
	{
		printf("\nInvalid address/ Address not supported \n");
		return -1;
	}
*/
	printf("\nstart connect \n");
	if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
	{
		printf("\nConnection Failed \n");
		return -1;
	}
	printf("\nfile open \n");

/*	if ( (fp = fopen(argv[2], "r")) == NULL){
		perror("fopen");
		exit(1);
	}

	while(!feof(fp)){
		numbytes = fread(buffer, sizeof(char), sizeof(buffer), fp);
		printf("fread %d bytes, ", numbytes);
		numbytes = write(sockfd, buffer, numbytes);
		printf("Sending %d bytesn",numbytes);
	}


	fclose(fp);
*/	close(sockfd);

	return 0;
}

