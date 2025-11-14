import java.net.*;
import javax.swing.*;
import javax.swing.table.DefaultTableModel;

public class CanEthernetMonitor extends JFrame {

    private DefaultTableModel tableModel;
    private JTable table;

    public CanEthernetMonitor() {
       
        setTitle("CAN to Ethernet Monitor");
        setSize(500, 400);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        
        String[] columns = {"CAN ID", "Data"};
        tableModel = new DefaultTableModel(columns, 0);
        table = new JTable(tableModel);

       
        add(new JScrollPane(table));

        setVisible(true);
       
        new Thread(this::listenForCanData).start();
    }

    private void listenForCanData() {
        int port = 5000; // UDP port
        byte[] buffer = new byte[1024];

        try (DatagramSocket socket = new DatagramSocket(port)) {
            System.out.println("Listening for CAN-Ethernet data on port " + port + "...");

            while (true) {
                DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                socket.receive(packet);

               
                String message = new String(packet.getData(), 0, packet.getLength()).trim();

                
                String canId = "Unknown";
                String data = "Unknown";

                if (message.contains("ID=")) {
                    int start = message.indexOf("ID=") + 3;
                    int end = message.indexOf(" ", start);
                    canId = (end > 0) ? message.substring(start, end) : message.substring(start);
                }

                if (message.contains("DATA=")) {
                    data = message.substring(message.indexOf("DATA=") + 5).trim();
                }

                final String idFinal = canId;
                final String dataFinal = data;

               
                SwingUtilities.invokeLater(() -> 
                    tableModel.addRow(new Object[]{idFinal, dataFinal})
                );
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(CanEthernetMonitor::new);
    }
}