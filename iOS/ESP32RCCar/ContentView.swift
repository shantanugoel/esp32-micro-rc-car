import SwiftUI
import CoreBluetooth
import Combine

// UUIDs matching the ESP32 GATT server
let SERVICE_UUID = CBUUID(string: "19b10000-e8f2-537e-4f6c-d104768a1214")
let MOTOR_CHAR_UUID = CBUUID(string: "19b10001-e8f2-537e-4f6c-d104768a1214")

enum DriveMode {
    case tank
    case arcade
}

struct ContentView: View {
    @StateObject private var bleManager = BLEManager()
    @State private var joystickPosition = CGPoint.zero
    @State private var driveMode: DriveMode = .tank
    @State private var throttle: Double = 0
    @State private var steering: Double = 0
    
    var body: some View {
        ZStack {
            // Background
            Color(UIColor.systemBackground)
                .ignoresSafeArea()
            
            VStack(spacing: 30) {
                // Header
                HStack {
                    VStack(alignment: .leading, spacing: 4) {
                        Text("ESP32 RC")
                            .font(.system(.title, design: .rounded))
                            .fontWeight(.bold)
                        
                        HStack(spacing: 6) {
                            Circle()
                                .fill(bleManager.isConnected ? Color.green : Color.red)
                                .frame(width: 8, height: 8)
                            Text(bleManager.statusText)
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }
                    }
                    
                    Spacer()
                    
                    HStack(spacing: 12) {
                        Button(action: {
                            withAnimation {
                                driveMode = driveMode == .tank ? .arcade : .tank
                            }
                            resetControls()
                        }) {
                            Image(systemName: driveMode == .tank ? "arrow.up.left.and.arrow.down.right" : "gauge")
                                .font(.system(size: 16))
                                .frame(width: 36, height: 36)
                                .background(Color(UIColor.tertiarySystemBackground))
                                .foregroundColor(.primary)
                                .cornerRadius(18)
                        }
                        
                        Button(action: {
                            if bleManager.isConnected {
                                bleManager.disconnect()
                            } else {
                                bleManager.startScanning()
                            }
                        }) {
                            Text(bleManager.isConnected ? "Disconnect" : "Connect")
                                .font(.subheadline)
                                .fontWeight(.medium)
                                .padding(.horizontal, 16)
                                .padding(.vertical, 8)
                                .background(bleManager.isConnected ? Color.red.opacity(0.1) : Color.blue.opacity(0.1))
                                .foregroundColor(bleManager.isConnected ? .red : .blue)
                                .cornerRadius(20)
                        }
                    }
                }
                .padding(.horizontal)
                .padding(.top)
                
                // Mode indicator
                Text(driveMode == .tank ? "Tank Drive" : "Arcade Drive")
                    .font(.caption)
                    .foregroundColor(.secondary)
                    .padding(.horizontal)
                
                Spacer()
                
                // Drive controls based on mode
                if driveMode == .tank {
                    JoystickView(position: $joystickPosition, onPositionChange: updateTankDrive)
                        .frame(width: 260, height: 260)
                        .transition(.scale.combined(with: .opacity))
                } else {
                    ArcadeControlsView(throttle: $throttle, steering: $steering, onUpdate: updateArcadeDrive)
                        .frame(width: 320, height: 320)
                        .transition(.scale.combined(with: .opacity))
                }
                
                Spacer()
                
                // Controls
                HStack(spacing: 40) {
                    // Spin Left
                    Button(action: {
                        bleManager.updateTargets(left: -40, right: 40)
                    }) {
                        VStack(spacing: 4) {
                            Image(systemName: "arrow.counterclockwise")
                                .font(.title2)
                            Text("Left")
                                .font(.caption2)
                                .fontWeight(.semibold)
                        }
                        .frame(width: 70, height: 70)
                        .background(Color(UIColor.secondarySystemBackground))
                        .foregroundColor(.primary)
                        .clipShape(Circle())
                    }
                    
                    // STOP
                    Button(action: {
                        bleManager.updateTargets(left: 0, right: 0)
                    }) {
                        Text("STOP")
                            .font(.headline)
                            .fontWeight(.heavy)
                            .frame(width: 90, height: 90)
                            .background(
                                LinearGradient(colors: [.red, .red.opacity(0.8)], startPoint: .top, endPoint: .bottom)
                            )
                            .foregroundColor(.white)
                            .clipShape(Circle())
                            .shadow(color: .red.opacity(0.4), radius: 10, x: 0, y: 5)
                    }
                    
                    // Spin Right
                    Button(action: {
                        bleManager.updateTargets(left: 40, right: -40)
                    }) {
                        VStack(spacing: 4) {
                            Image(systemName: "arrow.clockwise")
                                .font(.title2)
                            Text("Right")
                                .font(.caption2)
                                .fontWeight(.semibold)
                        }
                        .frame(width: 70, height: 70)
                        .background(Color(UIColor.secondarySystemBackground))
                        .foregroundColor(.primary)
                        .clipShape(Circle())
                    }
                }
                .padding(.bottom, 40)
            }
        }
    }
    
    func resetControls() {
        joystickPosition = .zero
        throttle = 0
        steering = 0
        bleManager.updateTargets(left: 0, right: 0)
    }
    
    func updateTankDrive(joystick: CGPoint) {
        let maxSpeed: Float = 50
        
        let throttleVal = Float(-joystick.y) * maxSpeed
        let steeringVal = Float(joystick.x) * maxSpeed * 0.7
        
        var left = throttleVal + steeringVal
        var right = throttleVal - steeringVal
        
        left = max(-127, min(127, left))
        right = max(-127, min(127, right))
        
        bleManager.updateTargets(left: Int8(left), right: Int8(right))
    }
    
    func updateArcadeDrive(throttle: Double, steering: Double) {
        let throttleVal = Float(throttle * 100)
        let steeringVal = Float(steering * 100)
        
        var left = throttleVal + steeringVal
        var right = throttleVal - steeringVal
        
        left = max(-127, min(127, left))
        right = max(-127, min(127, right))
        
        bleManager.updateTargets(left: Int8(left), right: Int8(right))
    }
}

// MARK: - Arcade Controls View
struct ArcadeControlsView: View {
    @Binding var throttle: Double
    @Binding var steering: Double
    let onUpdate: (Double, Double) -> Void
    
    @State private var isDraggingThrottle = false
    @State private var isDraggingSteering = false
    
    var body: some View {
        VStack(spacing: 40) {
            // Throttle Control (Vertical)
            HStack(spacing: 30) {
                VStack {
                    Text("THROTTLE")
                        .font(.caption2)
                        .fontWeight(.bold)
                        .foregroundColor(.secondary)
                    
                    ZStack {
                        Capsule()
                            .fill(Color(UIColor.secondarySystemBackground))
                            .frame(width: 60, height: 240)
                        
                        // Center line
                        Rectangle()
                            .fill(Color.gray.opacity(0.3))
                            .frame(width: 60, height: 2)
                        
                        // Thumb
                        Capsule()
                            .fill(
                                LinearGradient(
                                    colors: throttle > 0 ? [.green, .green.opacity(0.8)] : throttle < 0 ? [.orange, .orange.opacity(0.8)] : [.gray, .gray.opacity(0.8)],
                                    startPoint: .top,
                                    endPoint: .bottom
                                )
                            )
                            .frame(width: 50, height: 60)
                            .shadow(color: .black.opacity(0.2), radius: 4, y: 2)
                            .offset(y: CGFloat(-throttle) * 90)
                            .gesture(
                                DragGesture()
                                    .onChanged { value in
                                        isDraggingThrottle = true
                                        let normalized = -value.location.y / 90
                                        throttle = max(-1, min(1, normalized))
                                        onUpdate(throttle, steering)
                                    }
                                    .onEnded { _ in
                                        isDraggingThrottle = false
                                        withAnimation(.spring(response: 0.3, dampingFraction: 0.7)) {
                                            throttle = 0
                                        }
                                        onUpdate(0, steering)
                                    }
                            )
                    }
                    
                    HStack(spacing: 20) {
                        Text("REV")
                            .font(.caption2)
                            .foregroundColor(.orange)
                        Text("FWD")
                            .font(.caption2)
                            .foregroundColor(.green)
                    }
                }
                
                // Steering Control (Horizontal)
                VStack {
                    Text("STEERING")
                        .font(.caption2)
                        .fontWeight(.bold)
                        .foregroundColor(.secondary)
                    
                    ZStack {
                        Capsule()
                            .fill(Color(UIColor.secondarySystemBackground))
                            .frame(width: 200, height: 60)
                        
                        // Center line
                        Rectangle()
                            .fill(Color.gray.opacity(0.3))
                            .frame(width: 2, height: 60)
                        
                        // Thumb
                        Circle()
                            .fill(
                                LinearGradient(
                                    colors: [.blue, .blue.opacity(0.8)],
                                    startPoint: .leading,
                                    endPoint: .trailing
                                )
                            )
                            .frame(width: 50, height: 50)
                            .shadow(color: .black.opacity(0.2), radius: 4, y: 2)
                            .offset(x: CGFloat(steering) * 75)
                            .gesture(
                                DragGesture()
                                    .onChanged { value in
                                        isDraggingSteering = true
                                        let normalized = value.location.x / 75
                                        steering = max(-1, min(1, normalized))
                                        onUpdate(throttle, steering)
                                    }
                                    .onEnded { _ in
                                        isDraggingSteering = false
                                        onUpdate(throttle, steering)
                                    }
                            )
                    }
                    
                    HStack(spacing: 20) {
                        Text("LEFT")
                            .font(.caption2)
                            .foregroundColor(.secondary)
                        Spacer()
                        Text("RIGHT")
                            .font(.caption2)
                            .foregroundColor(.secondary)
                    }
                    .frame(width: 200)
                }
            }
            
            // Quick reset button
            Button(action: {
                withAnimation(.spring(response: 0.3, dampingFraction: 0.7)) {
                    throttle = 0
                    steering = 0
                }
                onUpdate(0, 0)
            }) {
                Image(systemName: "arrow.counterclockwise")
                    .font(.caption)
                    .foregroundColor(.secondary)
                    .padding(8)
                    .background(Color(UIColor.tertiarySystemBackground))
                    .clipShape(Circle())
            }
        }
    }
}

// MARK: - Joystick View
struct JoystickView: View {
    @Binding var position: CGPoint
    let onPositionChange: (CGPoint) -> Void
    @State private var dragOffset = CGSize.zero
    
    let knobRadius: CGFloat = 40
    
    var body: some View {
        GeometryReader { geometry in
            ZStack {
                // Base circle
                Circle()
                    .fill(Color(UIColor.secondarySystemBackground))
                    .shadow(color: Color.black.opacity(0.1), radius: 10, x: 0, y: 5)
                
                // Decorative rings
                Circle()
                    .stroke(Color.gray.opacity(0.1), lineWidth: 1)
                    .padding(20)
                
                // Knob
                Circle()
                    .fill(
                        LinearGradient(
                            gradient: Gradient(colors: [Color.blue, Color.blue.opacity(0.8)]),
                            startPoint: .topLeading,
                            endPoint: .bottomTrailing
                        )
                    )
                    .frame(width: knobRadius * 2, height: knobRadius * 2)
                    .shadow(color: Color.blue.opacity(0.3), radius: 8, x: 0, y: 4)
                    .overlay(
                        Circle()
                            .stroke(Color.white.opacity(0.3), lineWidth: 2)
                    )
                    .offset(x: dragOffset.width, y: dragOffset.height)
                    .gesture(
                        DragGesture(minimumDistance: 0)
                            .onChanged { value in
                                let maxOffset = (geometry.size.width / 2) - knobRadius
                                
                                var newX = value.translation.width
                                var newY = value.translation.height
                                
                                let distance = sqrt(newX * newX + newY * newY)
                                if distance > maxOffset {
                                    let scale = maxOffset / distance
                                    newX *= scale
                                    newY *= scale
                                }
                                
                                dragOffset = CGSize(width: newX, height: newY)
                                
                                let newPosition = CGPoint(
                                    x: newX / maxOffset,
                                    y: newY / maxOffset
                                )
                                position = newPosition
                                onPositionChange(newPosition)
                            }
                            .onEnded { _ in
                                withAnimation(.spring(response: 0.3, dampingFraction: 0.6)) {
                                    dragOffset = .zero
                                    position = .zero
                                }
                                onPositionChange(.zero)
                            }
                    )
            }
        }
        .aspectRatio(1, contentMode: .fit)
    }
}

// MARK: - BLE Manager
class BLEManager: NSObject, ObservableObject {
    @Published var isConnected = false
    @Published var statusText = "Disconnected"
    
    private var centralManager: CBCentralManager!
    private var peripheral: CBPeripheral?
    private var motorCharacteristic: CBCharacteristic?
    
    private let bleQueue = DispatchQueue(label: "com.esp32rccar.ble", qos: .userInteractive)
    private var commandTimer: DispatchSourceTimer?
    private let targetLock = NSLock()
    private var targetLeft: Int8 = 0
    private var targetRight: Int8 = 0
    
    override init() {
        super.init()
        centralManager = CBCentralManager(delegate: self, queue: bleQueue)
    }
    
    func startScanning() {
        guard centralManager.state == .poweredOn else {
            statusText = "Bluetooth not ready"
            return
        }
        statusText = "Scanning..."
        // Scan for all devices in case the ESP32 isn't advertising the UUID
        centralManager.scanForPeripherals(withServices: nil, options: nil)
    }
    
    func disconnect() {
        if let peripheral = peripheral {
            centralManager.cancelPeripheralConnection(peripheral)
        }
    }
    
    func updateTargets(left: Int8, right: Int8) {
        targetLock.lock()
        targetLeft = left
        targetRight = right
        targetLock.unlock()
    }
    
    private func startCommandLoop() {
        stopCommandLoop()
        
        let timer = DispatchSource.makeTimerSource(queue: bleQueue)
        timer.schedule(deadline: .now(), repeating: .milliseconds(50))
        timer.setEventHandler { [weak self] in
            self?.processCommandLoop()
        }
        timer.resume()
        commandTimer = timer
    }
    
    private func stopCommandLoop() {
        commandTimer?.cancel()
        commandTimer = nil
    }
    
    private func processCommandLoop() {
        guard let char = motorCharacteristic, let peripheral = peripheral, isConnected else { return }
        
        targetLock.lock()
        let left = targetLeft
        let right = targetRight
        targetLock.unlock()
        
        let data = Data([UInt8(bitPattern: left), UInt8(bitPattern: right)])
        peripheral.writeValue(data, for: char, type: .withoutResponse)
    }
}

extension BLEManager: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .poweredOn:
            statusText = "Ready - Tap Connect"
        case .poweredOff:
            statusText = "Bluetooth is off"
        case .unauthorized:
            statusText = "Bluetooth unauthorized"
        default:
            statusText = "Bluetooth unavailable"
        }
    }
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any], rssi RSSI: NSNumber) {
        let name = advertisementData[CBAdvertisementDataLocalNameKey] as? String ?? peripheral.name ?? "Unknown"
        
        var shouldConnect = false
        
        // Check for Service UUID
        if let serviceUUIDs = advertisementData[CBAdvertisementDataServiceUUIDsKey] as? [CBUUID],
           serviceUUIDs.contains(SERVICE_UUID) {
            shouldConnect = true
        }
        // Fallback: Check for "ESP32" in name
        else if name.localizedCaseInsensitiveContains("esp32") {
            shouldConnect = true
        }
        
        if shouldConnect {
            self.peripheral = peripheral
            centralManager.stopScan()
            statusText = "Connecting to \(name)..."
            centralManager.connect(peripheral, options: nil)
        }
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        peripheral.delegate = self
        peripheral.discoverServices([SERVICE_UUID])
        DispatchQueue.main.async {
            self.statusText = "Discovering services..."
        }
    }
    
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        stopCommandLoop()
        motorCharacteristic = nil
        DispatchQueue.main.async {
            self.isConnected = false
            self.statusText = "Disconnected"
        }
    }
    
    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        DispatchQueue.main.async {
            self.statusText = "Connection failed"
        }
    }
}

extension BLEManager: CBPeripheralDelegate {
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard let services = peripheral.services else { return }
        for service in services {
            if service.uuid == SERVICE_UUID {
                peripheral.discoverCharacteristics([MOTOR_CHAR_UUID], for: service)
            }
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        guard let chars = service.characteristics else { return }
        for char in chars {
            if char.uuid == MOTOR_CHAR_UUID {
                motorCharacteristic = char
                startCommandLoop()
                DispatchQueue.main.async {
                    self.isConnected = true
                    self.statusText = "Connected! 🎮"
                }
            }
        }
    }
}

#Preview {
    ContentView()
}
