#!/usr/bin/env python3
import can
import time
import struct

class Gim6010CANDiagnostic:
    def __init__(self, can_interface='can0', node_id=1):
        self.node_id = node_id
        self.bus = can.interface.Bus(channel=can_interface, bustype='socketcan', bitrate=1000000)
        
    def send_command(self, cmd_id, data=None):
        """Envoie une commande CAN au moteur"""
        can_id = (self.node_id << 5) | cmd_id
        if data is None:
            data = [0] * 8
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        self.bus.send(msg)
        print(f"Commande envoyée: ID=0x{can_id:03X}, Data={data}")
        
    def wait_for_message(self, cmd_id, timeout=5):
        """Attend un message spécifique"""
        can_id = (self.node_id << 5) | cmd_id
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.bus.recv(timeout=1)
            if msg and msg.arbitration_id == can_id:
                return msg
        return None
        
    def get_heartbeat(self):
        """Récupère le heartbeat du moteur"""
        print("=== DEMANDE HEARTBEAT ===")
        # Le heartbeat est envoyé périodiquement, on attend juste de le recevoir
        msg = self.wait_for_message(0x001, timeout=3)
        if msg:
            axis_state = msg.data[4]
            flags = msg.data[5]
            error = struct.unpack('<I', msg.data[0:4])[0]
            print(f"État: {axis_state}, Flags: {flags:08b}, Erreur: 0x{error:08X}")
            return axis_state, error
        else:
            print("❌ Aucun heartbeat reçu")
            return None, None
            
    def get_errors(self, error_type=0):
        """Récupère les erreurs (0=système, 1=moteur, 3=contrôleur, 4=encodeur)"""
        print(f"=== DEMANDE ERREURS (type={error_type}) ===")
        self.send_command(0x003, [error_type, 0, 0, 0, 0, 0, 0, 0])
        
        msg = self.wait_for_message(0x003, timeout=3)
        if msg:
            if error_type == 0:  # Système
                error = struct.unpack('<I', msg.data[0:4])[0]
                print(f"Erreur système: 0x{error:08X}")
            elif error_type == 1:  # Moteur
                error = struct.unpack('<Q', msg.data[0:8])[0]
                print(f"Erreur moteur: 0x{error:016X}")
            else:
                error = struct.unpack('<I', msg.data[0:4])[0]
                print(f"Erreur (type {error_type}): 0x{error:08X}")
            return error
        else:
            print("❌ Aucune réponse erreur")
            return 0
            
    def test_motor_calibration(self):
        """Teste la calibration moteur"""
        print("=== TEST CALIBRATION MOTEUR ===")
        self.send_command(0x007, [4, 0, 0, 0, 0, 0, 0, 0])  # AXIS_STATE_MOTOR_CALIBRATION
        
        # Surveillance pendant 15 secondes
        start_time = time.time()
        success = False
        while time.time() - start_time < 15:
            msg = self.wait_for_message(0x001, timeout=1)
            if msg:
                axis_state = msg.data[4]
                if axis_state == 1:  # IDLE
                    print("✅ Calibration moteur terminée")
                    success = True
                    break
                elif axis_state == 4:
                    print("Calibration en cours...")
                    
        if not success:
            print("❌ Timeout calibration moteur")
            # Forcer l'arrêt
            self.send_command(0x007, [1, 0, 0, 0, 0, 0, 0, 0])  # AXIS_STATE_IDLE
            
        return success
        
    def test_encoder_calibration(self):
        """Teste la calibration encodeur"""
        print("=== TEST CALIBRATION ENCODEUR ===")
        self.send_command(0x007, [7, 0, 0, 0, 0, 0, 0, 0])  # AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        
        # Surveillance pendant 20 secondes
        start_time = time.time()
        success = False
        while time.time() - start_time < 20:
            msg = self.wait_for_message(0x001, timeout=1)
            if msg:
                axis_state = msg.data[4]
                if axis_state == 1:  # IDLE
                    print("✅ Calibration encodeur terminée")
                    success = True
                    break
                elif axis_state == 7:
                    print("Calibration encodeur en cours...")
                    
        if not success:
            print("❌ Timeout calibration encodeur")
            # Forcer l'arrêt
            self.send_command(0x007, [1, 0, 0, 0, 0, 0, 0, 0])  # AXIS_STATE_IDLE
            
        return success
        
    def get_encoder_estimates(self):
        """Récupère les estimations de l'encodeur"""
        print("=== DEMANDE ESTIMATIONS ENCODEUR ===")
        self.send_command(0x009)  # Get_Encoder_Estimates
        
        msg = self.wait_for_message(0x009, timeout=3)
        if msg:
            pos = struct.unpack('<f', msg.data[0:4])[0]
            vel = struct.unpack('<f', msg.data[4:8])[0]
            print(f"Position: {pos:.2f} tours, Vitesse: {vel:.2f} tours/s")
            return pos, vel
        else:
            print("❌ Aucune réponse encodeur")
            return None, None
            
    def save_and_reboot(self):
        """Sauvegarde et redémarre"""
        print("=== SAUVEGARDE CONFIGURATION ===")
        self.send_command(0x01F)  # Save_Configuration
        time.sleep(3)
        
        print("=== REDÉMARRAGE ===")
        self.send_command(0x016)  # Reboot
        time.sleep(3)
        
    def full_diagnostic(self):
        """Effectue un diagnostic complet via CAN"""
        print("=== DÉBUT DIAGNOSTIC COMPLET VIA CAN ===")
        
        # 1. Vérifier le heartbeat
        print("\n1. Test de communication...")
        state, error = self.get_heartbeat()
        if state is None:
            print("❌ Impossible de communiquer avec le moteur")
            return False
            
        # 2. Vérifier les erreurs
        print("\n2. Vérification des erreurs...")
        system_error = self.get_errors(0)
        motor_error = self.get_errors(1)
        encoder_error = self.get_errors(4)
        
        # 3. Vérifier les estimations encodeur
        print("\n3. Test de l'encodeur...")
        pos, vel = self.get_encoder_estimates()
        
        # 4. Test calibration moteur
        print("\n4. Test calibration moteur...")
        motor_ok = self.test_motor_calibration()
        
        # 5. Test calibration encodeur
        print("\n5. Test calibration encodeur...")
        encoder_ok = self.test_encoder_calibration()
        
        # 6. Sauvegarde et redémarrage
        print("\n6. Sauvegarde et redémarrage...")
        self.save_and_reboot()
        
        # 7. Résultat final
        print("\n=== RÉSULTAT DU DIAGNOSTIC ===")
        print(f"Communication: ✅")
        print(f"Erreurs système: {'❌' if system_error != 0 else '✅'}")
        print(f"Erreurs moteur: {'❌' if motor_error != 0 else '✅'}")
        print(f"Erreurs encodeur: {'❌' if encoder_error != 0 else '✅'}")
        print(f"Calibration moteur: {'✅' if motor_ok else '❌'}")
        print(f"Calibration encodeur: {'✅' if encoder_ok else '❌'}")
        
        overall_ok = (system_error == 0 and motor_error == 0 and 
                     encoder_error == 0 and motor_ok and encoder_ok)
        
        if overall_ok:
            print("\n🎉 DIAGNOSTIC: TOUT EST OK !")
        else:
            print("\n💥 DIAGNOSTIC: DES PROBLÈMES ONT ÉTÉ DÉTECTÉS")
            
        return overall_ok

# Utilisation
if __name__ == "__main__":
    try:
        # Initialiser avec l'ID du moteur
        diagnostic = Gim6010CANDiagnostic(can_interface='can0', node_id=1)
        
        # Lancer le diagnostic complet
        success = diagnostic.full_diagnostic()
        
        if success:
            print("\nLe moteur est prêt à être utilisé !")
        else:
            print("\nDes actions correctives sont nécessaires.")
            
    except KeyboardInterrupt:
        print("\n⚠️  Interruption utilisateur")
    except Exception as e:
        print(f"\n❌ Erreur: {e}")
    finally:
        diagnostic.bus.shutdown()
