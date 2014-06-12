#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <string>
#include "japi.h"
//socket tcp
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

//log file
#include <fstream>

//julius recognize result msg
#include <hark_msgs/HarkJuliusSrc.h>
#include <hark_msgs/HarkJuliusSrcVal.h>

//string
#include <iostream>
#include <typeinfo>

//boost
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>


using namespace std;

//Log File
std::ofstream ofs( "/home/koike/Dropbox/HRI/workspace/recogResultPublisher/resultTxt/result.txt" );

//Julius Recog Data
vector<hark_msgs::HarkJuliusSrc> RECOG_WORD_LIST;

static char rbuf[MAXLINELEN]; ///< Local workarea for message string handling

//HarkJuliusSrc Index
const int SOURCEID_INDEX = 1;
const int AZIMUTH_INDEX = 3;
const int ELEVATION_INDEX = 5;
const int SEC_INDEX = 7;
const int USEC_INDEX = 9;

//HarkJuliusSrcVal Index
//const int SCORE_INDEX = 5;
const int WORD_INDEX = 17;
const int CLASSID_INDEX = 19;
const int PHONE_INDEX = 21;
const int CM_INDEX = 23;

//Score Threshold -> TODO:read by xml
//const Int SCORE_THRESHOLD = 0

/** 
 * <JA>
 * サーバーからメッセージを一行読み込みバッファに格納する．
 * 末尾の改行コードは削除される．
 * 
 * @param sd [in] 受信ソケット
 * @param buf [out] 受信したメッセージを格納するバッファ
 * @param maxlen [in] @a buf の最大長
 * 
 * @return @a buf へのポインタ, あるいはエラー時はNULLを返す．
 * </JA>
 * <EN>
 * Receive message from server for one line, and store it to buffer.
 * The newline character at end will be stripped.
 * 
 * @param sd [in] socket descriptor to receive data
 * @param buf [out] buffer to store the received message string.
 * @param maxlen [in] maximum allowed length of @a buf
 * 
 * @return pointer equal to @a buf, or NULL if error.
 * </EN>
 */
char *
do_receive(int sd, char *buf, int maxlen)
{
	int cnt;
	char *p;

	p = buf;
	while(1) {
		#ifdef WINSOCK
				cnt = recv(sd, p, 1, 0);
		#else
				cnt = read(sd, p, 1);
		#endif
		if (cnt <= 0) return NULL;		/* eof or error */
		if (*p == '\n' && p > buf) {
			*p = '\0';
			break;
		} else {
			if (++p >= buf + maxlen) {
				fprintf(stderr,"Error: do_receive: line too long (> %d)\n", maxlen);
				exit(1);
			}
		}
	}
	return buf;
}



/** 
 * <JA>
 * 
 * ""で囲まれたパラメータを抽出
 * 
 * @param str 認識結果の文字列
 * 
 * </JA>
 * 
 **/
//~ vector<string> getParam(string str){
	//~ vector<string> params;
	//~ boost::algorithm::split(params, str, boost::is_any_of("\""));
	//~ return params;
//~ }

/** 
 * <JA>
 * 
 * julius_mftから出力された認識結果の文字列を解析して、Julius用のメッセージファイルに格納
 * 
 * @param recogStr 認識結果の文字列
 * 
 * </JA>
 * 
 **/
hark_msgs::HarkJuliusSrc getRecogMsg(string recogStr){

	hark_msgs::HarkJuliusSrc msg;
	msg.id = -1;

	if(recogStr.substr(0,11)=="<SOURCEINFO"){
		vector<string> params;
		boost::algorithm::split(params, recogStr, boost::is_any_of("\""));
		hark_msgs::HarkJuliusSrc recogMsg;
		recogMsg.id = boost::lexical_cast<int>(params.at(SOURCEID_INDEX));
		recogMsg.azimuth = boost::lexical_cast<float>(params.at(AZIMUTH_INDEX));
		recogMsg.elevation = boost::lexical_cast<float>(params.at(ELEVATION_INDEX));
		recogMsg.sec = boost::lexical_cast<int>(params.at(SEC_INDEX));
		recogMsg.usec = boost::lexical_cast<int>(params.at(USEC_INDEX));
		RECOG_WORD_LIST.push_back(recogMsg);
	}

	else if(recogStr.substr(0,9)=="<RECOGOUT"){
		vector<string> params;
		boost::algorithm::split(params, recogStr, boost::is_any_of("\""));
		int recogId = boost::lexical_cast<int>(params.at(SOURCEID_INDEX));
		vector<hark_msgs::HarkJuliusSrc>::iterator it = RECOG_WORD_LIST.begin();

		for( it = RECOG_WORD_LIST.begin(); it != RECOG_WORD_LIST.end(); ++it ){
			if((*it).id == recogId){
				msg = *it;
				RECOG_WORD_LIST.erase(it);
				break;
			}
		}

		hark_msgs::HarkJuliusSrcVal val;
		//TODO:put score and cut data under threshold
		val.word = params.at(WORD_INDEX);
		val.classid = boost::lexical_cast<int>(params.at(CLASSID_INDEX));
		val.phone = params.at(PHONE_INDEX);
		val.cm = boost::lexical_cast<float>(params.at(CM_INDEX));
		vector<hark_msgs::HarkJuliusSrcVal> valList;
		valList.push_back(val);
		msg.src = valList;
	}
	return msg;
}


string getRecogStr(int sd)
{
	string recogResult ="";
	
	while(do_receive(sd, rbuf, MAXLINELEN) != NULL) {
		if (rbuf == NULL || (rbuf[0] == '.' && rbuf[1] == '\0')) break;
		recogResult += string(rbuf) + "\n";
	}
	
	ofs << recogResult;
	fflush(stdout);

	return recogResult;
}

// print recognized word for stdout
void printRecogMsg(hark_msgs::HarkJuliusSrc &recogMsg){
	cout << endl;
	cout << "id:" << recogMsg.azimuth << endl;
	cout << "azimuth:" << recogMsg.azimuth << endl;
	cout << "word:" << recogMsg.src.at(0).word << endl;
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	//socket
	struct sockaddr_in server;
	int sock;

	//julius recognize message file
	hark_msgs::HarkJuliusSrc recogMsg;
 
	/* ソケットの作成 */
	sock = socket(AF_INET, SOCK_STREAM, 0);

	/* 接続先指定用構造体の準備 */
	server.sin_family = AF_INET;
	server.sin_port = htons(10500);
	server.sin_addr.s_addr = inet_addr("127.0.0.1");

	/* サーバに接続 */
	if(connect(sock, (struct sockaddr *)&server, sizeof(server))==-1){
		cout << "server connect error" << endl;
		return -1;
	}
	else{
		cout << "server connect success" << endl;
	}

	//ros node initialize
	ros::init(argc, argv, "RecogResult");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<hark_msgs::HarkJuliusSrc>("HarkJuliusSrc", 1000);
	ros::Rate loop_rate(10);

	//ros topic publish
	while (ros::ok())
	{
		recogMsg = getRecogMsg(getRecogStr(sock));
		printRecogMsg(recogMsg);
		if( recogMsg.id >= 0){
			chatter_pub.publish(recogMsg);
		}

		ros::spinOnce();
	}

	 /* socketの終了 */
	close(sock);

	return 0;
}
