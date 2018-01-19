#include <errno.h>
#include <unistd.h>
#include <fcntl.h>

#include <sys/time.h>
#include <stdio.h>

int main()
{
	FILE *fp0 = NULL;
	char Buf[512];
	
	/*��ʼ��Buf*/
	strcpy(Buf,"Initial Mem is char dev! write to BUF");
	printf("BUF: %s\n\n",Buf);
	
	/*���豸�ļ�*/
	fp0 = fopen("/dev/mymemdev","r+");
	if (fp0 == NULL)
	{
		perror("fopen error");
		return 0;
	}

	/*д���豸*/
	fwrite(Buf, sizeof(Buf), 1, fp0);
	
	sleep(5);
	/*���¶�λ�ļ�λ�ã�˼��û�и�ָ����кκ��)*/
	fseek(fp0,0,SEEK_SET);
	
	/*���Buf*/
	strcpy(Buf,"String in Buf is changed to NULL!");
	printf("BUF: %s\n",Buf);
	
	/*�����豸*/
	fread(Buf, sizeof(Buf), 1, fp0);
	
	/*�����*/
	printf("BUF: %s\n",Buf);
	
	return 0;	

}
