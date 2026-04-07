//
//  ViewController.swift
//  ZephyrusTelemetryUI
//
//  Created by Marc D. Nichitiu on 3/25/26.
//

import Cocoa
import UniformTypeIdentifiers

class ViewController: NSViewController, NSWindowDelegate, NSTextFieldDelegate {

    private var scriptProcess: Process?
    private var outputPipe: Pipe?

    @IBOutlet weak var toggleUI: NSButton!
    @IBOutlet weak var saveLog: NSButton!
    @IBOutlet weak var pathField: NSTextField!
    @IBOutlet weak var clearLog: NSButton!

    var isUIOn: Bool = false
    
    //@IBOutlet weak var log: NSTextField!
    @IBOutlet var log: NSTextView!
    
    private let pathDefaultsKey = "LastPathFieldValue"
    
    @IBAction func saveLogAction(_ sender: Any) {
        let textToSave = log?.string ?? ""
        let panel = NSSavePanel()
        panel.title = "Save Log"
        panel.allowedContentTypes = [.plainText]
        panel.nameFieldStringValue = "UI_log.txt"
        panel.canCreateDirectories = true

        panel.beginSheetModal(for: self.view.window!) { [weak self] response in
            guard response == .OK, let url = panel.url else { return }
            do {
                try textToSave.data(using: .utf8)?.write(to: url)
            } catch {
                // Present an alert on failure
                let alert = NSAlert()
                alert.messageText = "Failed to Save Log"
                alert.informativeText = error.localizedDescription
                alert.alertStyle = .warning
                alert.beginSheetModal(for: self?.view.window ?? NSApp.mainWindow!)
            }
        }
    }
    
    @IBAction func clearLog(_ sender: Any) {
        log.string = "Log"
    }
    
    
    
    @IBAction func toggleUIAction(_     sender: Any) {
        if isUIOn {
            stopUIProcess()
        }
        else {
            
            log.string = "Log at \(Date().formatted())"
            // If the UI is not on, instantiate it according to
            // Start the script "script.sh"
            // Harvest all text output into a text box, referenced by `log`
            let process = Process()
            let pipe = Pipe()
            outputPipe = pipe
            pipe.fileHandleForReading.readabilityHandler = nil

            process.standardOutput = pipe
            process.standardError = pipe
            process.currentDirectoryURL = URL(fileURLWithPath: pathField.stringValue)
            
            // Persist the chosen path
            savePathToDefaults()

            var env = ProcessInfo.processInfo.environment
            env["PYTHONUNBUFFERED"] = "1"
            process.environment = env
                        
            process.executableURL = URL(fileURLWithPath: "/opt/homebrew/bin/python3")
            process.arguments = ["-u", "UI.py"]
            

            // If the script requires a shell to run, you can use /bin/bash -lc script instead. Here we execute directly.
            do {
                try process.run()
            } catch {
                DispatchQueue.main.async { [weak self] in
                    self?.log?.string += (self?.log?.string.isEmpty == true ? "" : "\n") + "Failed to start script: \(error.localizedDescription)"
                }
                return
            }

            process.terminationHandler = { [weak self] _ in
                DispatchQueue.main.async {
                    self?.handleUIProcessExit()
                }
            }

            scriptProcess = process
            isUIOn = true
            if let button = toggleUI { button.title = "Stop" }

            // Read output asynchronously and append to log
            pipe.fileHandleForReading.readabilityHandler = { [weak self] handle in
                let data = handle.availableData
                guard !data.isEmpty, let chunk = String(data: data, encoding: .utf8) else { return }
                DispatchQueue.main.async {
                    if let logView = self?.log {
                        let timeStamp = Date().formatted(.iso8601.year().month().day().dateSeparator(.dash).time(includingFractionalSeconds: true))
                        let prefix = logView.string.isEmpty ? "\(timeStamp) " : "\n\(timeStamp) "
                        logView.string += prefix + chunk.trimmingCharacters(in: .newlines)
                        // Scroll to bottom to show latest output
                        let endRange = NSRange(location: (logView.string as NSString).length, length: 0)
                        logView.scrollRangeToVisible(endRange)
                    }
                }
            }
        }
    }

    private func stopUIProcess() {
        guard let process = scriptProcess else {
            handleUIProcessExit()
            return
        }

        if process.isRunning {
            process.terminate()
        }
    }

    private func handleUIProcessExit() {
        scriptProcess?.terminationHandler = nil
        scriptProcess = nil

        if let pipe = outputPipe {
            pipe.fileHandleForReading.readabilityHandler = nil
            try? pipe.fileHandleForReading.close()
        }
        outputPipe = nil

        isUIOn = false
        toggleUI?.title = "Start"
    }
    
    private func savePathToDefaults() {
        let value = pathField.stringValue
        UserDefaults.standard.set(value, forKey: pathDefaultsKey)
    }
    
    func controlTextDidEndEditing(_ obj: Notification) {
        guard let textField = obj.object as? NSTextField, textField == pathField else { return }
        savePathToDefaults()
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()

        // Do any additional setup after loading the view.
        DispatchQueue.main.async { [weak self] in
            self?.view.window?.delegate = self
        }

        // Configure pathField delegate and load last saved value
        pathField.delegate = self
        if let savedPath = UserDefaults.standard.string(forKey: pathDefaultsKey), !savedPath.isEmpty {
            pathField.stringValue = savedPath
        }
    }

    override var representedObject: Any? {
        didSet {
        // Update the view, if already loaded.
        }
    }

    func windowWillClose(_ notification: Notification) {
        savePathToDefaults()
        stopUIProcess()
        NSApp.terminate(nil)
    }


}
