#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <mutex>

void error(const char *msg)
{
    if (errno == ECONNRESET || errno == ECONNREFUSED) printf("Got reset\n");
    perror(msg);
    exit(0);
}

int sockfd;

void sighandler(int sig) {
    if (sig == 141 || sig == SIGPIPE) {
        perror("Got pipe error");
        return;
    }
    if (errno == ECONNRESET || errno == ECONNREFUSED) printf("Got reset\n");
    printf("Got signal %d\n", sig);
    close(sockfd);
    exit(0);
}

int clientMain2(const char * hostname, int portno, char * buffer, bool * ready, std::mutex * mtx)
{
    mtx->lock();
    int sockfd, n;

    struct sockaddr_in serv_addr;
    struct hostent *server;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error("ERROR opening socket");
    server = gethostbyname(hostname);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *)&serv_addr,sizeof(serv_addr)) < 0) {
        if (errno == ECONNRESET || errno == ECONNREFUSED || errno == EINVAL) {
            close(sockfd);
            return 2;
        }
        error("ERROR connecting");
    } else fprintf(stderr, "Connected.\n");
    bzero(buffer,256);
    signal(SIGINT, sighandler);
    signal(141, sighandler);
    signal(SIGPIPE, sighandler);
    mtx->unlock();
    usleep(1000);
    while (1) {
        while (!*ready);
	mtx->lock();
        printf("Locked client thread\n");
        n = write(sockfd,buffer,256);
        if (n < 0) {
            perror("Got error");
            close(sockfd);
            return 2;
        } else fprintf(stderr, "Sent data\n");
        bzero(buffer,256);
        *ready = false;
        printf("Unlocking client thread\n");
        mtx->unlock();
        usleep(1000);
        //n = read(sockfd,buffer,255);
        //if (n < 0)
        //     error("ERROR reading from socket");
        //printf("%s\n",buffer);
    }
    return 0;
}

int clientMain(const char * hostname, int portno, char * buffer, bool * ready, std::mutex * mtx) {
    while (1) {
        clientMain2(hostname, portno, buffer, ready, mtx);
        printf("Disconnected.");
    }
}
