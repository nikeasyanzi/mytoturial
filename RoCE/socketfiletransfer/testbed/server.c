#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#define PORT 1234
#include <time.h>

int main(int argc, char const *argv[])
{
	int server_fd, new_socket, valread;
	struct sockaddr_in address;
	int opt = 1;
	int addrlen = sizeof(address);
	char buffer[1024] = {0};
	FILE *fp;
	int numbytes=0;
	int testcount=10;
	int i;
	// Creating socket file descriptor
	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
	{
		perror("socket failed");
		exit(EXIT_FAILURE);
	}

	// Forcefully attaching socket to the port 8080
	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
				&opt, sizeof(opt)))
	{
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons( PORT );

	// Forcefully attaching socket to the port 8080
	if (bind(server_fd, (struct sockaddr *)&address,
				sizeof(address))<0)
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}

	printf(" start to listen\n");

	for (i = 1; i < testcount; i++) {
		printf("%dth trail\n",i);
		if (listen(server_fd, 3) < 0)
		{ 
			perror("listen");
			exit(EXIT_FAILURE);
		}
		printf(" accepting\n");

		if ((new_socket = accept(server_fd, (struct sockaddr *)&address,
						(socklen_t*)&addrlen))<0)
		{
			perror("accept");
			exit(EXIT_FAILURE);
		}


		if ( (fp = fopen("received_file", "wb")) == NULL){
			perror("fopen");
			exit(1);
		}

		clock_t tic = clock();

		while(1){
			numbytes = read(new_socket, buffer, sizeof(buffer));
			//		printf("read %d bytes, ", numbytes);
			if(numbytes == 0){
				break;
			}
			numbytes = fwrite(buffer, sizeof(char), numbytes, fp);
			//		printf("fwrite %d bytesn", numbytes);
		}
		clock_t toc = clock();

		printf("Elapsed: %f seconds\n", (double)(toc - tic) / CLOCKS_PER_SEC);

		fclose(fp);
	}
	close(server_fd);

	return 0;
}

