// qFFTFilter.h
// Main plugin class that CloudCompare recognizes and loads
// This follows the standard CloudCompare plugin interface

#ifndef Q_FFT_FILTER_PLUGIN_HEADER
#define Q_FFT_FILTER_PLUGIN_HEADER

// CloudCompare's standard plugin interface
#include "ccStdPluginInterface.h"

// Forward declaration of the dialog window
class qFFTFilterDialog;

//! FFT Filter Plugin
/** This plugin allows users to:
    1. Select a scalar field representing time-series data
    2. Compute and visualize its FFT spectrum
    3. Apply frequency-domain filtering (band-pass)
    4. Generate filtered scalar fields
**/
class qFFTFilter : public QObject, public ccStdPluginInterface
{
    // Qt meta-object system (required for signals/slots)
    Q_OBJECT
    
    // Plugin interface declarations
    Q_INTERFACES(ccPluginInterface ccStdPluginInterface)
    
    // Qt plugin metadata (tells Qt this is a plugin)
    // Use the resource path defined in qFFTFilter.qrc
    Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qFFTFilter" FILE "../info.json")

public:
    //! Default constructor
    /** This is called when CloudCompare loads the plugin **/
    explicit qFFTFilter(QObject* parent = nullptr);
    
    //! Destructor
    /** Clean up resources when plugin is unloaded **/
    ~qFFTFilter() override;

    // --- Inherited from ccStdPluginInterface ---
    
    //! Called when plugin action is triggered
    /** This is the main entry point when user clicks the plugin menu item
        @param action The QAction that was triggered (each plugin can have multiple actions)
    **/
    void onNewSelection(const ccHObject::Container& selectedEntities) override;
    
    //! Returns list of actions this plugin provides
    /** These appear in CloudCompare's menu system
        @return List of QAction pointers
    **/
    QList<QAction*> getActions() override;

    // --- Inherited from ccPluginInterface ---
    
    //! Plugin type (Standard, GL, I/O)
    /** Standard plugins add processing/analysis features **/
    CC_PLUGIN_TYPE getType() const override { return CC_STD_PLUGIN; }
    
    //! Plugin name (shown in menus)
    QString getName() const override { return "FFT Filter"; }
    
    //! Plugin description
    QString getDescription() const override 
    { 
        return "Compute FFT of scalar fields and apply frequency-domain filtering"; 
    }
    
    //! Plugin icon (shown in toolbar/menu)
    QIcon getIcon() const override;

protected:
    //! Register this plugin's actions with CloudCompare's UI
    /** Called during plugin initialization **/
    void registerActions();

private:
    //! The main action that opens the FFT filter dialog
    QAction* m_action;
    
    //! Pointer to the filter dialog window
    /** We keep this as a member so it persists across multiple uses **/
    qFFTFilterDialog* m_dialog;
};

#endif // Q_FFT_FILTER_PLUGIN_HEADER