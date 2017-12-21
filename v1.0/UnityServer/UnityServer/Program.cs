using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net;
using System.Net.Sockets;
namespace UnityServer
{
    
    class Program
    {
        const int MAX_USER = 10;
        static Socket receiver;
        static Socket sender;
        static bool isOpen = false;
        static string receivedMSG;
        static void Main(string[] args)
        {
            UnityClient Unity ;
            UserClient User ;
            int n = 0;
            while (true)
            {
                string cmd;
                Console.Write(">> ");
                cmd = Console.ReadLine();
                if (cmd == "open") openServer();
                else if (cmd == "help") showCmd();
                else if (cmd == "conuser")
                {
                    User = (UserClient)receiver.Accept() ;
                    Console.WriteLine(User.SocketType);
                    while (User.formatReceived() == null)
                    {
                    }
                    Console.WriteLine("Connected from " + User.formatReceived());
                }
                else if (cmd == "conunity")
                {
                    Unity = (UnityClient)sender.Accept();
                    Console.WriteLine("Connected to Unity");
                }
                else if (cmd == "status") showStatus();
                else if (cmd == "send")
                {
                    if (sender.Connected)
                    {
                        Unity = (UnityClient)sender.Accept();
                        Console.Write("input command : ");
                        string msg = Console.ReadLine();
                        Unity.sendFormatted(msg);
                    }
                    else
                    {
                        Console.Write("fuck you bro");
                    }
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
            string IPaddr = Dns.GetHostEntry(hostName).AddressList[0].ToString();
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
        }

        static void openServer()
        {
            string hostName = Dns.GetHostName();
            string IPaddr = Dns.GetHostEntry(hostName).AddressList[2].ToString();
            if (isOpen)
            {
                receiver.Disconnect(true);
                sender.Disconnect(true);
            }
            receiver = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            receiver.Bind(new IPEndPoint(IPAddress.Any, 22999));
            receiver.Listen(100);
            if (receiver.IsBound) Console.WriteLine("Receiver is Ready : " +IPaddr+" :22999");
            sender = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            sender.Bind(new IPEndPoint(IPAddress.Any, 23000));
            sender.Listen(100);
            if (sender.IsBound) Console.WriteLine("Sender is Ready : " + IPaddr + " :23000");
            isOpen = true;
        }
        static void connectUnity()
        {
            
        }

        static void connectUser()
        {
            
        }
        static void receiving(UserClient User)
        {
            receivedMSG = User.formatReceived();
        }
    }
    
    
    public class UnityClient : Socket
    {
        public UnityClient(AddressFamily addressFamily, SocketType socketType, ProtocolType protocolType) : base(addressFamily,socketType,protocolType)
        {
            //NULL
        }

        public void sendFormatted(string msg)
        {
            Send(Encoding.ASCII.GetBytes(msg));
        }

    } 

    public class UserClient : Socket
    {
        public UserClient(AddressFamily addressFamily, SocketType socketType, ProtocolType protocolType) : base(addressFamily,socketType,protocolType)
        {
            //NULL
        }

        public string formatReceived()
        {
            string tmp;
            byte[] buffer = new byte[ReceiveBufferSize];
            if (ReceiveBufferSize == 0) return null;
            Receive(buffer);
            tmp = Encoding.UTF8.GetString(buffer);
            return tmp;
        } 
    }
}
