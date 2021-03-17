#include "ns3/log.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/lora-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/forwarder-helper.h"
#include "ns3/network-server-helper.h"
#include "ns3/periodic-sender-helper.h"
#include <iostream>
#include <fstream>
#include <ns3/spectrum-module.h>
#include <ns3/okumura-hata-propagation-loss-model.h>
#include "ns3/flow-monitor-helper.h"

using namespace ns3;
using namespace lorawan;
NS_LOG_COMPONENT_DEFINE ("Teste");

struct device{
    int SF;
};
struct spf{
    int S;
    int R;
    Time delay;
};

//Instantiate of data structures
std::vector<device> d;
std::vector<spf> spread;
std :: map <uint64_t, int> pacote_sf;
std :: map <uint64_t, Time> pacote_ds;
std :: map <uint64_t, Time> pacote_dr;
std::vector<double> distances;
std::vector<double> axis_x;

//Count Sent Packet per SF
void PacketTraceDevice(Ptr<Packet const> pacote){
    uint32_t id =  Simulator::GetContext ();
    pacote_sf.insert({pacote->GetUid(), d[id].SF});
    Time sendTime = Simulator::Now ();
    pacote_ds.insert({pacote->GetUid(), sendTime});
    spread[d[id].SF].S++;   
}

//Count Received Packet per SF
void PacketTraceGW(Ptr<Packet const> pacote){
    u_int64_t pkid = pacote->GetUid();
    int sf = pacote_sf.at(pkid);
    Time receivedTime = Simulator :: Now ();
    Time sent = pacote_ds.at(pkid);
    pacote_dr.insert({pkid, receivedTime - sent});
    spread[sf].R++;
}

//Get Average Distance of Devices From Gateway per SF
void AverageDistances(NodeContainer endDevices, NodeContainer gateways){
    for(NodeContainer::Iterator gw = gateways.Begin (); gw != gateways.End (); ++gw){
        Ptr<MobilityModel> mobModelG = (*gw)->GetObject<MobilityModel>();
        
        for (NodeContainer::Iterator node = endDevices.Begin (); node != endDevices.End (); ++node)
        {
            Ptr<MobilityModel> mobModel = (*node)->GetObject<MobilityModel>();
            double position = mobModel->GetDistanceFrom(mobModelG);  
            uint32_t nodeId = (*node)->GetId();
            if(d[nodeId].SF == 5){
            distances[5] = distances[5]+position;
            }else if(d[nodeId].SF == 4){    
            distances[4] = distances[4]+position;
            }else if(d[nodeId].SF == 3){
            distances[3] = distances[3]+position;
            }else if(d[nodeId].SF == 2){
            distances[2] = distances[2]+position;
            }else if(d[nodeId].SF == 1){
            distances[1] = distances[1]+position;
            }else{
            distances[0] = distances[0]+position;
            }
        }
    }    
}

//Get Distance of Devices on X-Axis
void DistanceX(NodeContainer endDevices){
  for (NodeContainer::Iterator node = endDevices.Begin (); node != endDevices.End (); ++node)
  {
        Ptr<MobilityModel> mobModel = (*node)->GetObject<MobilityModel>();
        Vector3D pos = mobModel->GetPosition();
        uint32_t nodeId = (*node)->GetId();
        if(d[nodeId].SF == 5){
          axis_x[5] = axis_x[5]+ fabs(pos.x);
        }else if(d[nodeId].SF == 4){    
          axis_x[4] = axis_x[4]+ fabs(pos.x);
        }else if(d[nodeId].SF == 3){
          axis_x[3] = axis_x[3]+ fabs(pos.x);
        }else if(d[nodeId].SF == 2){
          axis_x[2] = axis_x[2]+ fabs(pos.x);
        }else if(d[nodeId].SF == 1){
          axis_x[1] = axis_x[1]+ fabs(pos.x);
        }else{
          axis_x[0] = axis_x[0]+ fabs(pos.x);
        }
  }      
}
//Print position of devices, distance from gateway and positions (x,y) per SF
void Print(NodeContainer endDevices, NodeContainer gateways,  Ptr<PropagationDelayModel> delay, double interval)
{    
    std::string logFile = "resultados.txt";
    // open log file for output
    std::ofstream os;
    os.open (logFile.c_str ());
    std::vector<double> x;
    std::vector<double> y;
    std::vector<std::string> color;

    for(NodeContainer::Iterator gw = gateways.Begin (); gw != gateways.End (); ++gw){
        uint32_t gwId = (*gw)->GetId(); 
        Ptr<MobilityModel> mobModelG = (*gw)->GetObject<MobilityModel>();
        Vector3D posgw = mobModelG->GetPosition();
        
        for (NodeContainer::Iterator node = endDevices.Begin (); node != endDevices.End (); ++node)
        {
        Ptr<MobilityModel> mobModel = (*node)->GetObject<MobilityModel>();
        Vector3D pos = mobModel->GetPosition();
        double position = mobModel->GetDistanceFrom(mobModelG);  
        uint32_t nodeId = (*node)->GetId();
        // Prints position and velocities
        x.push_back(pos.x);
        y.push_back(pos.y);
        if(d[nodeId].SF == 0){
          color.push_back("SF12");
        }else if(d[nodeId].SF == 1){
          color.push_back("SF11");
        }else if(d[nodeId].SF == 2){
          color.push_back("SF10");
        }else if(d[nodeId].SF == 3){
          color.push_back("SF9");
        }else if(d[nodeId].SF == 4){
          color.push_back("SF8");
        }else{
          color.push_back("SF7");
        }
      
        os << "Posição Node "
            << nodeId << ", "
            << pos.x << ":" << pos.y << ":" << pos.z << "\n";
        os << "Posição Gateway " << gwId << ", " << posgw.x << ":" << posgw.y << ":" << posgw.z << "\n";
        os << "DISTANCE:" << position << "\n" << "\n"; 
        }
     }

    for(double i: x){
       os << i << ", "; 
    }
    os << "\n";
    for(double i: y){
       os << i << ", "; 
    }
    os << "\n";
    for(auto i: color){
       os << i << ", "; 
    }
    os << "\n";     
    os.close();
    Simulator::Schedule(Seconds(interval), &Print, endDevices, gateways, delay, interval);

}

int n_devices = 100;
int n_gateways = 1;
double length = 4000;
int n_times = 1;

int main (int argc, char *argv[]){

    CommandLine cmd;
    cmd.AddValue ("n_devices", "Number of end devices to include in the simulation", n_devices);
    cmd.AddValue ("length", "The radius of the area to simulate", length);
    cmd.AddValue ("n_gateways", "Number of gateways to include in the simulation", n_gateways);
    cmd.AddValue ("n_times", "Number of times to run the simulation", n_times);
    
    for(int n = 0; n < n_times; n++){
      srand(time(0));
      int seed = rand();
      RngSeedManager::SetSeed (seed);
      RngSeedManager::SetRun (7);
      LogComponentEnable ("Teste", LOG_LEVEL_ALL);
      //Chamadas de funções da classe com endereços da memória dos componentes, tamanho do pacote e alguns tempos computados (?)
      //LogComponentEnable("LoraPhy", LOG_LEVEL_ALL);
      //Só chamada de funções da classe EndDevice
      //LogComponentEnable("EndDeviceLoraPhy", LOG_LEVEL_ALL);
      //Chamadas de funções com endereços da memórias como parâmetros;
      //LogComponentEnable("GatewayLoraPhy", LOG_LEVEL_ALL);
      //Avisa se encontrou interferência, em qual canal, a energia, bem como intervalo de tempo
      //LogComponentEnable("LoraInterferenceHelper", LOG_LEVEL_ALL);
      //Chamada de função com endereço e algum ID (?)
      //LogComponentEnable("LorawanMac", LOG_LEVEL_ALL);
      //Chamada de funções da classe + frequência do canal atual e tempo de espera pra tal canal
      //LogComponentEnable("EndDeviceLorawanMac", LOG_LEVEL_ALL);
      //Chama da funções da classe + frequência, quanto tipos de transmissões faltam, se é mensagem confirmada, (replyDataRate, m_rx1DrOffset e m_dataRate (?)) 
      //LogComponentEnable("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
      //Chamada de funções da classe com endereços da memória como parâmetros
      //LogComponentEnable("GatewayLorawanMac", LOG_LEVEL_ALL);
      //Chamada de funções da classe com endereços da memória, tempo da mensagem no ar, m_aggregatedDutyCycle(?), tempo atual e quando a próxima transmissão vai ser permitida na sub-banda; 
      //LogComponentEnable("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
      //Chamada de funções da classe com endereços da memória
      //LogComponentEnable("LogicalLoraChannel", LOG_LEVEL_ALL);
      //Avisa quando cria as camadas PHY e MAC, bem como cada dispositivo e gateway e suas posições
      //LogComponentEnable("LoraHelper", LOG_LEVEL_ALL);
      //Chamada de funções da classe com endereços da memória
      //LogComponentEnable("LoraPhyHelper", LOG_LEVEL_ALL);
      //Chamada de funções da classe com endereços da memória
      //LogComponentEnable("LorawanMacHelper", LOG_LEVEL_ALL);
      //Chamada de funções da classe com endereços da memória e o intervalo de tempo da aplicação criada
      //LogComponentEnable("PeriodicSenderHelper", LOG_LEVEL_ALL);
      //Chamada de funções da classe com endereços da memória, avisa do id do dispositivo (evento) e que enviou o pacote
      //LogComponentEnable("PeriodicSender", LOG_LEVEL_ALL);
      //Chamada de funções da classe
      //LogComponentEnable("LorawanMacHeader", LOG_LEVEL_ALL);
      //Chamada de funções da classe e algumas variáveis (ACK, fpending, foptslen, fcnt)
      //LogComponentEnable("LoraFrameHeader", LOG_LEVEL_ALL);
      //Chamada de funções da classe com endereços da memória, avisa a abertura de janela do dispositivo e se encontrou gateway disponível 
      //LogComponentEnable("NetworkScheduler", LOG_LEVEL_ALL);
      //id do dispositivo + propagação: txPower (transmissor), rxPower (receptor), distância acho q do gateway, delay da mensagem
      //LogComponentEnable ("LoraChannel", LOG_LEVEL_INFO);
      //Funções da classe e endereços dos componentes passados como parâmetros
      //LogComponentEnable("NetworkServer", LOG_LEVEL_ALL);
      //Chamada de funções da classe com endereços da memória
      //LogComponentEnable("NetworkStatus", LOG_LEVEL_ALL);
      //Chamada de funções da classe com endereços da memória
      //LogComponentEnable("NetworkController", LOG_LEVEL_ALL);
      //Pega endereço de pacote e mostra de qual dispositivo foi enviado e se foi recebido pelo gateway
      //LogComponentEnable("LoraPacketTracker", LOG_LEVEL_ALL);

      //Inicialization
      d.resize(n_devices);
      spread.resize(6);
      distances.resize(6);
      axis_x.resize(6);
      NS_LOG_INFO ("\nTesting...");
      
      //Create Propagation Loss
      Ptr<NakagamiPropagationLossModel> nakagami = CreateObject<NakagamiPropagationLossModel>();
      nakagami->SetAttribute("m0", DoubleValue(1));
      nakagami->SetAttribute("m1",DoubleValue(1));
      nakagami->SetAttribute("m2",DoubleValue(1));
      Ptr<OkumuraHataPropagationLossModel> loss = CreateObject<OkumuraHataPropagationLossModel>();
      loss->SetAttribute("Frequency",DoubleValue(868e6));
      loss->SetNext(nakagami);
      loss->Initialize();
      //Create channel
      Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();
      Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);

      //Helpers
      // Create the LoraPhyHelper
      LoraPhyHelper phyHelper = LoraPhyHelper ();
      phyHelper.SetChannel (channel);
      // Create the LorawanMacHelper
      LorawanMacHelper macHelper = LorawanMacHelper ();
      // Create the LoraHelper
      LoraHelper helper = LoraHelper ();
      helper.EnablePacketTracking ();

      MobilityHelper mobility;
      mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator", "rho", DoubleValue (length),
                                          "X", DoubleValue (0.0), "Y", DoubleValue (0.0));
      mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
      //EndDevices
      NodeContainer endDevices;
      endDevices.Create (n_devices);
      /*MobilityHelper mobilityEd;
      Ptr<UniformRandomVariable> randT = CreateObject<UniformRandomVariable> ();
      Ptr<ListPositionAllocator> nodePositionList = CreateObject<ListPositionAllocator>();
    double lengthMax = length;
    for(uint32_t nodePositionsAssigned = 0; nodePositionsAssigned < 100; nodePositionsAssigned++){
      double x,y;
      do{
        x = randT->GetInteger(0,lengthMax);
        y = randT->GetInteger(0,lengthMax);
      }
      while ((x-length)*(x-length)+(y-length)*(y-length) > lengthMax*lengthMax);
      nodePositionList->Add (Vector (x,y,1.0));
    }
    mobilityEd.SetPositionAllocator (nodePositionList);
    mobilityEd.SetMobilityModel ("ns3::ConstantPositionMobilityModel");*/
    mobility.Install (endDevices);
      
      // Make it so that nodes are at a certain height > 0
      for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
      {
          Ptr<MobilityModel> m = (*j)->GetObject<MobilityModel> ();
          Vector position = m->GetPosition ();
          position.z = 1.0;
          //std :: cout << position << std :: endl;
          m->SetPosition (position);
      }
          
      phyHelper.SetDeviceType (LoraPhyHelper::ED);
      macHelper.SetDeviceType (LorawanMacHelper::ED_A);
      helper.Install (phyHelper, macHelper, endDevices);
      
      //Gateway
      NodeContainer gateways;
      gateways.Create (n_gateways);
      Ptr<ListPositionAllocator> positionAllocGw = CreateObject<ListPositionAllocator> ();
      positionAllocGw->Add (Vector (0.0, 0.0, 50.0));
      mobility.SetPositionAllocator (positionAllocGw);
      mobility.Install(gateways);

      phyHelper.SetDeviceType (LoraPhyHelper::GW);
      macHelper.SetDeviceType (LorawanMacHelper::GW);
      macHelper.SetRegion (LorawanMacHelper::EU);
      helper.Install (phyHelper, macHelper, gateways);

      
      //SpreadingFactor
      std::vector<int> sf = macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);
      for (std::vector<int>::const_iterator i = sf.begin(); i != sf.end(); ++i)
          std::cout << *i << ' ';
      std :: cout << "\n";
      // Connect trace sources
      for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
      {
          Ptr<Node> node = *j;
          uint32_t id =  node->GetId();
          Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
          Ptr<ClassAEndDeviceLorawanMac> mac = loraNetDevice->GetMac ()->GetObject<ClassAEndDeviceLorawanMac> ();
          uint8_t DR =  mac->GetDataRate();
          if (unsigned (DR) == 5){
              d[id].SF = 5;
          }else if (unsigned(DR) == 4){
              d[id].SF = 4;
          }else if (unsigned(DR) == 3){
              d[id].SF = 3;
          }else if (unsigned(DR) == 2){
              d[id].SF = 2;
          }else if (unsigned(DR) == 1){
              d[id].SF = 1;
          }else{
              d[id].SF = 0;
          }
                  
          mac->TraceConnectWithoutContext("SentNewPacket", MakeCallback(&PacketTraceDevice));

      }
      for (NodeContainer::Iterator j = gateways.Begin (); j != gateways.End (); ++j)
      {
          Ptr<Node> node = *j;
          Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
          Ptr<LorawanMac> mac = loraNetDevice->GetMac()->GetObject<LorawanMac>();
          mac->TraceConnectWithoutContext("ReceivedPacket", MakeCallback(&PacketTraceGW));
      }


      //NetworkServer
      NodeContainer networkServers;
      networkServers.Create (1);
      NetworkServerHelper networkServerHelper;
      networkServerHelper.SetGateways (gateways);
      networkServerHelper.SetEndDevices (endDevices);
      networkServerHelper.Install (networkServers);
      // Install the Forwarder application on the gateways
      ForwarderHelper forwarderHelper;
      forwarderHelper.Install (gateways);

      Time appStopTime = Hours(24);
      PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
      appHelper.SetPeriod (Seconds (120));
      appHelper.SetPacketSize (51);
      Ptr<RandomVariableStream> rv = CreateObjectWithAttributes<UniformRandomVariable> (
          "Min", DoubleValue (0), "Max", DoubleValue (10));
      ApplicationContainer appContainer = appHelper.Install (endDevices);

      // Start simulation
      Simulator::Stop (appStopTime);
      Simulator::Schedule(Seconds(0.00), &Print, endDevices, gateways, delay, 10.0);

      Simulator::Run ();

      Simulator::Destroy ();

      //Pacotes enviados e recebidos
      LoraPacketTracker &tracker = helper.GetPacketTracker ();
      //helper.DoPrintDeviceStatus(endDevices, gateways, "resultados.txt");
      int iterator = n_devices;
      for (NodeContainer::Iterator j = gateways.Begin (); j != gateways.End (); ++j){
          std :: vector <int> output = tracker.CountPhyPacketsPerGw(Seconds(0), appStopTime, iterator);
          std :: cout << "GwID " << iterator << "\nReceived: " << output.at(1) << "\nInterfered: " << output.at(2)
          << "\nNoMoreReceivers: " << output.at(3) << "\nUnderSensitivity: " << output.at(4) << "\nLost: " << output.at(5)
          << "\n" << "\n";
          iterator += 1;
      }    
      std::string s =  tracker.CountMacPacketsGlobally (Seconds (0), appStopTime);
      std::stringstream ss(s);
      std::string item;
      std::vector<std::string> splittedStrings;
      while (std::getline(ss, item, ' '))
      {
          splittedStrings.push_back(item);
      }

      double sent = std::stod(splittedStrings[0]);
      double receiv = std::stod(splittedStrings[1]);
      double PER = ( sent - receiv )/receiv;
      NS_LOG_INFO ("\nNumber of Packets Sent And Received");
      std :: cout << sent << ' ' << receiv << "\n";

      NS_LOG_INFO ("\nPacket error rate ");
      std :: cout << PER << "\n";

      
      for(auto i = pacote_dr.begin(); i != pacote_dr.end(); i++){
        int SF = pacote_sf[i->first];
        spread[SF].delay += i->second;
      }

      NS_LOG_INFO ("\nNumber of packets sent and received per SF");
      for (std::vector<spf>::iterator i = spread.begin(); i!= spread.end(); ++i){
        if(i->delay != Time(0)){
          std :: cout << i->S << " " << i->R <<  " " << (i->delay/i->R).GetMilliSeconds()  << " " << i->delay/i->R  << " " << i->delay << "\n";
          i->S = i->R  = 0;
          i->delay = Time(0);
        }
        else{  
          std :: cout << i->S << " " << i->R << "\n";
          i->S = i->R = 0;
          i->delay = Time(0);
        }  
      }

      NS_LOG_INFO ("\nAverage Distance of Devices Per SF");
      sf[5] = sf[5] + sf[6];
      sf[6] = 0;
      AverageDistances(endDevices, gateways);
      int num = 5;
      for (double i : distances){  
        std :: cout << i/sf[num] << "\n";
        num--;
      }

      NS_LOG_INFO ("\nAverage Axis X of Devices Per SF");
      DistanceX(endDevices);
      num = 5;
      for (double i : axis_x){  
        std :: cout << i/sf[num] << "\n";
        num--;
      }

      //Cleaning
      pacote_sf.clear();
      pacote_ds.clear();
      pacote_dr.clear();
      d.clear();
      spread.clear();
      distances.clear();
      axis_x.clear();
    }
    return 0;
}
