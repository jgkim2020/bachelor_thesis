using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;
namespace UnityServer
{

    class Program
    {
        const int MAX_USER = 10;
        static Socket receiver;
        static Socket sender;
        static Socket Unity;
        static Socket User;
        static bool isOpen = false;
        static bool userConn = false;
        static bool unityConn = false;
        static string receivedMSG;
        static void Main(string[] args)
        {

            int n = 0;
            while (true)
            {
                string cmd;
                Console.Write(">> ");
                cmd = Console.ReadLine();
                if (cmd == "open") openServer();
                else if (cmd == "help") showCmd();
                else if (cmd == "process")
                {
                    Thread th = new Thread(receivingTransmitting);
                    th.Start();
                }
                else if (cmd == "conuser")
                {
                    User = receiver.Accept();
                    Console.WriteLine(User.SocketType);
                    Console.WriteLine("Connected from " + formatReceived(User));
                    userConn = true;
                }
                else if (cmd == "conunity")
                {
                    Unity = sender.Accept();
                    Console.WriteLine(Unity.SocketType);
                    Console.WriteLine("Connected from " + formatReceived(Unity));
                    unityConn = true;
                }
                else if (cmd == "status") showStatus();
                else if (cmd == "send")
                {
                    Console.Write("input command : ");
                    string msg = Console.ReadLine();
                    sendFormatted(Unity, msg);
                }
                else if (cmd == "clear")
                {
                    Console.Clear();
                }
                else
                {
                    Console.Write("Invaild command");
                }

            }
        }

        static void showStatus()
        {
            string hostName = Dns.GetHostName();
            string IPaddr = Dns.GetHostEntry(hostName).AddressList[2].ToString();
            if (isOpen)
            {
                if (receiver.Connected) Console.WriteLine("Receiver is Connected : " + IPaddr + ":22999");
                else Console.WriteLine("Receiver is Ready : " + IPaddr + ":22999");
                if (sender.Connected) Console.WriteLine("Sender is Connected : " + IPaddr + ":23000");
                else Console.WriteLine("Sender is Ready : " + IPaddr + ":23000");
            }
            else
            {
                Console.WriteLine("Server is not opened!");
            }
        }
        static void showCmd()
        {
            Console.WriteLine("open : open or reset server");
            Console.WriteLine("send : send command to Unity");
            Console.WriteLine("help : help you");
            Console.WriteLine("conuser : connect to user");
            Console.WriteLine("conunity : connect to unity");
            Console.WriteLine("process : start transmitting thread");
            Console.WriteLine("clear : clear the console screen");
        }
        
        static void openServer()
        {
            string hostName = Dns.GetHostName();
            string IPaddr = Dns.GetHostEntry(hostName).AddressList[2].ToString();
            // Close existing session
            if (isOpen)
            {
                receiver.Disconnect(true);
                sender.Disconnect(true);
            }
            // Configure receiver
            receiver = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            receiver.Bind(new IPEndPoint(IPAddress.Any, 22999));
            receiver.Listen(100);
            if (receiver.IsBound) Console.WriteLine("Receiver is Ready : " + IPaddr + " :22999");
            // Configure sender
            sender = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            sender.Bind(new IPEndPoint(IPAddress.Any, 23000));
            sender.Listen(100);
            if (sender.IsBound) Console.WriteLine("Sender is Ready : " + IPaddr + " :23000");
            isOpen = true;
        }
        static void receivingTransmitting()
        {
            while (isOpen)
            {
                if (userConn)
                {
                    receivedMSG = formatReceived(User);
                    Console.WriteLine(receivedMSG);
                }
                if (unityConn) sendFormatted(Unity, receivedMSG);
                Thread.Sleep(1);
            }
        }

        static void sendFormatted(Socket s, string msg)
        {
            s.Send(Encoding.ASCII.GetBytes(msg));
        }

        static string formatReceived(Socket s)
        {
            string tmp;
            byte[] buffer = new byte[100];
            if (s.ReceiveBufferSize == 0) return null;
            s.Receive(buffer);
            tmp = Encoding.UTF8.GetString(buffer);
            return tmp;
        }
    }
}
    
   
