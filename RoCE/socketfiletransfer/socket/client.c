#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#define PORT 8080

int main(int argc, char const *argv[])
{
	struct sockaddr_in address;
	int sockfd = 0;
	FILE * fp;
	int valread;
	struct sockaddr_in serv_addr;
	char buffer[1024] = {0};
	int numbytes;
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		printf("\n Socket creation error \n");
		return -1;
	}

	memset(&serv_addr, '0', sizeof(serv_addr));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(PORT);


//	bzero( &(address.sin_zero), 8 );

	// Convert IPv4 and IPv6 addresses from text to binary form
	if(inet_pton(AF_INET, argv[1], &serv_addr.sin_addr)<=0)
	{
		printf("\nInvalid address/ Address not supported \n");
		return -1;
	}

	if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
	{
		printf("\nConnection Failed \n");
		return -1;
	}


	if ( (fp = fopen("test", "r")) == NULL){
		perror("fopen");
		exit(1);
	}


	while(!feof(fp)){
		numbytes = fread(buffer, sizeof(char), sizeof(buffer), fp);
//		printf("fread %d bytes, ", numbytes);
		numbytes = write(sockfd, buffer, numbytes);
//		printf("Sending %d bytesn",numbytes);
	}


	fclose(fp);
	close(sockfd);

	return 0;
}

