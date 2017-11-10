#include "bm_dispatcher.h"
#include "bm_tcp_datastream.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>

#include "server.h"
#include "bm_pose2d.h"

//communication range between robots
float Communication_Range=10.0;

static const int RAB_PACKAGE = 0;
static const int ABS_POS_PACKAGE = 1;

/****************************************/
/****************************************/

/*
 * This is set to 0 at the beginning to mean: we're not done.
 * When set to 1, this means: the program must finish.
 * The value of this variable is set by a signal handler.
 */
static int done = 0;
int SidI, RidI;

/*
 * Active threads
 */
static int active_threads = 0;

/****************************************/
/****************cyj*********************/
extern pose2d pose_opt[];

pose2d *p2d;

void computeRB ( char *Sid, char *Rid, float *range, float *bearing )
{
    //server();
    p2d = pose_opt;
    SidI = strtol ( Sid + 1, NULL, 10 );
    RidI = strtol ( Rid + 1, NULL, 10 );
    *range = sqrt ( pow ( p2d[SidI].x - p2d[RidI].x, 2 ) + pow ( p2d[SidI].y - p2d[RidI].y, 2 ) );
    float dely = p2d[SidI].y - p2d[RidI].y;
    float delx = p2d[SidI].x - p2d[RidI].x;


    float delTheta = atan2 ( dely,delx );


    *bearing = delTheta - p2d[RidI].theta;// change the angle to reciever coordinate system
    *bearing=remainder ( *bearing,2.0*M_PI );



    fprintf ( stdout, "Sender id: %d - (%.2fm,%.2f degrees) - Receiver id : %d\n", SidI, *range, *bearing * 180 / 3.14, RidI );
    /*   if (RidI == 2) {  printf("receiver degree:%.2f \n",p2d[RidI].theta );
       fprintf(stdout, "dely: %.2f  delx: %.2f; delTheta: %.2fdegrees; ReceiverDegreees: %.2f; bearing: %.2f degrees\n", dely, delx, delTheta*180/3.14,p2d[RidI].theta*180/3.14, *bearing*180/3.14);}
    */
}

/* for testing show

	if (SidI==2){
	printf("receiver degree:%.2f \n",p2d[RidI].theta );
	fprintf(stdout, "dely: %.2f  delx: %.2f; delTheta: %.2fdegrees; ReceiverDegreees: %.2f; bearing: %.2f degrees\n", dely, delx, delTheta*180/3.14,p2d[RidI].theta*180/3.14, *bearing*180/3.14);}*/
//return;


void bm_dispatcher_broadcast ( bm_dispatcher_t dispatcher,
                               bm_datastream_t stream,
                               uint8_t* data )
{
//   printf("+++++++++++++++%s\n", data);
//   printf("+++++++++++++++%s\n", *data);
    pthread_mutex_lock ( &dispatcher->datamutex );
    bm_datastream_t cur = dispatcher->streams;
    ssize_t sent;

     p2d = pose_opt;

    float range=0,bearing=0;
    while ( cur ) {
        //if ( cur != stream ) {
            //printf("Now is in broadcast function and before server\n");

	    char subbuff[2];
	    memcpy( subbuff, &stream->id[1], 2 );
	    int khepera_id = strtol(subbuff, NULL, 10);

	    //printf("Khepera_id: %d", khepera_id);

	    float x = pose_opt[khepera_id].x;
	    float y = pose_opt[khepera_id].y;
	    float theta = pose_opt[khepera_id].theta;
	    float angle = 0.0;

            computeRB ( stream->id, cur->id, &range, &bearing );

            //fprintf(stdout, "Sender id: %d - (%.2fm,%.2frad) - Receiver id : %d\n", stream->id, range, bearing, cur->id);
            //if distance between two robots is out of range, do not send
            //if ( range<=Communication_Range ) {
                //construct_package ( data, dispatcher->msg_len, RAB_PACKAGE, -1, range, bearing, 0 );

                size_t shift = sizeof ( uint16_t );
                memcpy ( data + shift, &range, sizeof ( float ) );
                shift += sizeof ( float );
                memcpy ( data + shift, &bearing, sizeof ( float ) );
                shift += sizeof ( float );
                memcpy ( data + shift, &angle, sizeof ( float ) );
                shift += sizeof ( float );

                memcpy ( data + shift, &khepera_id, sizeof ( int ) );
		shift += sizeof ( int );
                memcpy ( data + shift, &x, sizeof ( float ) );
                shift += sizeof ( float );
                memcpy ( data + shift, &y, sizeof ( float ) );
                shift += sizeof ( float );
                memcpy ( data + shift, &theta, sizeof ( float ) );

		printf("Khepera (%d): %f, %f", khepera_id, x, y);


                sent = cur->send ( cur, data, dispatcher->msg_len );
            //}
            if ( sent < dispatcher->msg_len ) {
                fprintf ( stderr,
                          "sent %zd bytes instead of %zu to %s: %s\n",
                          sent,
                          dispatcher->msg_len,
                          cur->descriptor,
                          cur->status_desc );
            }
        //}
        // printf("Now is after computerRB***sender ID: %d***\n", SidI);
        cur = cur->next;

    }
    pthread_mutex_unlock ( &dispatcher->datamutex );
}



/****************cyj*********************/
/****************************************/


struct bm_dispatcher_thread_data_s {
    bm_dispatcher_t dispatcher;
    bm_datastream_t stream;
};
typedef struct bm_dispatcher_thread_data_s* bm_dispatcher_thread_data_t;

void* bm_dispatcher_thread ( void* arg )
{
    /* Get thread data */

    bm_dispatcher_thread_data_t data = ( bm_dispatcher_thread_data_t ) arg;

    /* Wait for start signal */
    pthread_mutex_lock ( &data->dispatcher->startmutex );
    ++active_threads;
    while ( data->dispatcher->start == 0 )
        pthread_cond_wait ( &data->dispatcher->startcond,
                            &data->dispatcher->startmutex );
    pthread_mutex_unlock ( &data->dispatcher->startmutex );
    /* Execute logic */
    uint8_t* buf = ( uint8_t* ) calloc ( data->dispatcher->msg_len, 1 );
    //for counting  in while
    //int count=0;
    while ( !done ) {
        /* Receive data */
        if ( data->stream->recv ( data->stream,
                                  buf,
                                  data->dispatcher->msg_len ) <= 0 ) {
            // printf("Now is in if of while with broadcast\n");

            /* Error receiving data, exit */
            fprintf ( stderr, "%s: exiting\n", data->stream->descriptor );
            pthread_mutex_lock ( &data->dispatcher->startmutex );
            --active_threads;
            pthread_mutex_unlock ( &data->dispatcher->startmutex );
            return NULL;
        }


        /* Broadcast data */
        bm_dispatcher_broadcast(data->dispatcher, data->stream, buf);
	// add absolute position
        //bm_dispatcher_absolute_position ( data->dispatcher, data->stream, buf );
    }
    /* All done */
    pthread_mutex_lock ( &data->dispatcher->startmutex );
    --active_threads;
    pthread_mutex_unlock ( &data->dispatcher->startmutex );
    return NULL;
}

/****************************************/
/****************************************/

bm_dispatcher_t bm_dispatcher_new()
{
    bm_dispatcher_t d = ( bm_dispatcher_t ) malloc ( sizeof ( struct bm_dispatcher_s ) );
    d->streams = NULL;
    d->stream_num = 0;
    d->start = 0;
    d->msg_len = 0;
    if ( pthread_cond_init ( &d->startcond, NULL ) != 0 ) {
        fprintf ( stderr, "Error initializing the start condition variable: %s\n",
                  strerror ( errno ) );
        free ( d );
        return NULL;
    }
    if ( pthread_mutex_init ( &d->startmutex, NULL ) != 0 ) {
        fprintf ( stderr, "Error initializing the start mutex: %s\n",
                  strerror ( errno ) );
        free ( d );
        return NULL;
    }
    if ( pthread_mutex_init ( &d->datamutex, NULL ) != 0 ) {
        fprintf ( stderr, "Error initializing the data mutex: %s\n",
                  strerror ( errno ) );
        free ( d );
        return NULL;
    }
    return d;
}

/****************************************/
/****************************************/

void bm_dispatcher_destroy ( bm_dispatcher_t d )
{
    pthread_cond_destroy ( &d->startcond );
    pthread_mutex_destroy ( &d->startmutex );
    pthread_mutex_destroy ( &d->datamutex );
    bm_datastream_t cur = d->streams;
    bm_datastream_t next;
    while ( cur ) {
        next = cur->next;
        cur->destroy ( cur );
        cur = next;
    }
    free ( d );
}

/****************************************/
/****************************************/

int bm_dispatcher_stream_add ( bm_dispatcher_t d,
                               const char* s )
{
    char* ws = strdup ( s );
    char* saveptr;
    /* Get stream id */
    char* tok = strtok_r ( ws, ":", &saveptr );
    if ( !tok ) {
        fprintf ( stderr, "Can't parse '%s'\n", s );
        free ( ws );
        return 0;
    }
    /* Make sure id has not been already used */
    for ( bm_datastream_t cur = d->streams;
            cur != NULL;
            cur = cur->next ) {
        if ( strcmp ( tok, cur->id ) == 0 ) {
            fprintf ( stderr, "'%s': id '%s' already in use by '%s'\n",
                      s,
                      tok,
                      cur->descriptor );
            free ( ws );
            return 0;
        }
    }
    /* Get stream type */
    tok = strtok_r ( NULL, ":", &saveptr );
    if ( !tok ) {
        fprintf ( stderr, "Can't parse '%s'\n", s );
        free ( ws );
        return 0;
    }
    /* Create the stream */
    bm_datastream_t stream;
    if ( strcmp ( tok, "tcp" ) == 0 ) {
        /* Create new TCP stream */
        stream = ( bm_datastream_t ) bm_tcp_datastream_new ( s );
    } else if ( strcmp ( tok, "bt" ) == 0 ) {
        /* Create new Bluetooth stream */
        fprintf ( stderr, "'%s': Bluetooth streams not supported yet\n", s );
        free ( ws );
        return 0;
    } else if ( strcmp ( tok, "xbee" ) == 0 ) {
        /* Create new XBee stream */
        fprintf ( stderr, "'%s': XBee streams not supported yet\n", s );
        free ( ws );
        return 0;
    } else {
        fprintf ( stderr, "'%s': Unknown stream type '%s'\n", s, tok );
        free ( ws );
        return 0;
    }
    /* Attempt to connect */
    if ( !stream->connect ( stream ) ) {
        fprintf ( stderr, "'%s': Connection error: %s\n", s, stream->status_desc );
        bm_datastream_destroy ( stream );
        free ( ws );
        return 0;
    }
    /* Add a thread dedicated to this stream */

    bm_dispatcher_thread_data_t info = ( bm_dispatcher_thread_data_t ) malloc (sizeof ( struct bm_dispatcher_thread_data_s ) );

    info->dispatcher = d;
    info->stream = stream;
    if ( pthread_create ( &stream->thread, NULL, &bm_dispatcher_thread, info ) != 0 ) {
        fprintf ( stderr, "'%s': Can't create thread: %s\n", s, strerror ( errno ) );
        bm_datastream_destroy ( stream );
        free ( info );
        free ( ws );
        return 0;
    }
    /* Add stream at the beginning of the list */
    if ( d->streams != NULL ) {
        stream->next = d->streams;
    }
    d->streams = stream;
    ++d->stream_num;
    /* Wrap up */
    fprintf ( stdout, "Added stream '%s'\n", s );
    free ( ws );
    return 1;
}

/****************************************/
/****************************************/

void sighandler ( int sig )
{
    /* The program is done */
    fprintf ( stdout, "Termination requested\n" );
    done = 1;
}

void bm_dispatcher_execute ( bm_dispatcher_t d )
{
    /* Set signal handlers */
    signal ( SIGTERM, sighandler );
    signal ( SIGINT, sighandler );
    /* Start all threads */
    pthread_mutex_lock ( &d->startmutex );
    d->start = 1;
    pthread_mutex_unlock ( &d->startmutex );
    pthread_cond_broadcast ( &d->startcond );
    /* Wait for done signal */
    while ( !done ) {
        sleep ( 1 );
        pthread_mutex_lock ( &d->startmutex );
        if ( active_threads == 0 ) {
            done = 1;
        }
        pthread_mutex_unlock ( &d->startmutex );
    }
    /* Cancel all threads */
    for ( bm_datastream_t s = d->streams;
            s != NULL;
            s = s->next ) {
        pthread_cancel ( s->thread );
    }
    /* Wait for all threads to be done */
    for ( bm_datastream_t s = d->streams;
            s != NULL;
            s = s->next ) {
        pthread_join ( s->thread, NULL );
    }
}

/****************************************/
/****************************************/
