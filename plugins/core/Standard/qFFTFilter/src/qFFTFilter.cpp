// qFFTFilter.cpp
// Implementation of the main plugin class

#include "qFFTFilter.h"
#include "qFFTFilterDialog.h"

// CloudCompare includes
#include <ccPointCloud.h>
#include <ccGLWindow.h>
#include <ccMainAppInterface.h>

// Qt includes
#include <QAction>
#include <QIcon>
#include <QMessageBox>

// Constructor: Initialize the plugin
qFFTFilter::qFFTFilter(QObject* parent)
    : QObject(parent)
    , ccStdPluginInterface(":/CC/plugin/qFFTFilter/info.json")
    , m_action(nullptr)
    , m_dialog(nullptr)
{
    // Plugin initialization happens here
    // CloudCompare will call this when loading the plugin
}

// Destructor: Clean up resources
qFFTFilter::~qFFTFilter()
{
    // Delete the dialog if it exists
    if (m_dialog)
    {
        delete m_dialog;
        m_dialog = nullptr;
    }
}

// Called when user selects entities in CloudCompare
void qFFTFilter::onNewSelection(const ccHObject::Container& selectedEntities)
{
    // Update the dialog if it's open
    if (m_dialog && m_dialog->isVisible())
    {
        // Pass the selected entities to the dialog
        m_dialog->updateSelectedEntities(selectedEntities);
    }
    
    // Enable/disable the action based on selection
    if (m_action)
    {
        // Enable only if at least one point cloud is selected
        bool hasPointCloud = false;
        for (ccHObject* entity : selectedEntities)
        {
            if (entity && entity->isA(CC_TYPES::POINT_CLOUD))
            {
                hasPointCloud = true;
                break;
            }
        }
        m_action->setEnabled(hasPointCloud);
    }
}

// Return the list of actions this plugin provides
QList<QAction*> qFFTFilter::getActions()
{
    // If actions haven't been created yet, create them
    if (m_action == nullptr)
    {
        registerActions();
    }
    
    // Return a list containing our action(s)
    QList<QAction*> actions;
    actions.push_back(m_action);
    return actions;
}

// Get the plugin icon
QIcon qFFTFilter::getIcon() const
{
    // Load icon from resources (or return default icon)
    // You can create an icon.png in the images/ folder
    return QIcon(":/CC/plugin/qFFTFilter/images/icon.png");
}

// Register actions with CloudCompare
void qFFTFilter::registerActions()
{
    // Create the main action
    m_action = new QAction("FFT Filter", this);
    m_action->setToolTip("Compute FFT spectrum and apply frequency filtering");
    m_action->setIcon(getIcon());
    
    // Connect the action to open the dialog
    connect(m_action, &QAction::triggered, this, [this]()
    {
        // Get the main application interface
        ccMainAppInterface* app = m_app;
        if (!app)
        {
            QMessageBox::critical(nullptr, "Error", "Main application interface not available");
            return;
        }
        
        // Get currently selected entities
        const ccHObject::Container& selectedEntities = app->getSelectedEntities();
        
        // Check if we have at least one point cloud
        ccPointCloud* cloud = nullptr;
        for (ccHObject* entity : selectedEntities)
        {
            if (entity && entity->isA(CC_TYPES::POINT_CLOUD))
            {
                cloud = static_cast<ccPointCloud*>(entity);
                break;
            }
        }
        
        if (!cloud)
        {
            QMessageBox::information(
                reinterpret_cast<QWidget*>(app->getMainWindow()),
                "No Point Cloud",
                "Please select a point cloud with scalar fields to use FFT Filter."
            );
            return;
        }
        
        // Check if cloud has scalar fields
        if (cloud->getNumberOfScalarFields() == 0)
        {
            QMessageBox::information(
                reinterpret_cast<QWidget*>(app->getMainWindow()),
                "No Scalar Fields",
                "The selected point cloud has no scalar fields. "
                "FFT Filter requires at least one scalar field."
            );
            return;
        }
        
        // Create dialog if it doesn't exist
        if (!m_dialog)
        {
            m_dialog = new qFFTFilterDialog(app, reinterpret_cast<QWidget*>(app->getMainWindow()));
        }
        
        // Update dialog with current selection
        m_dialog->updateSelectedEntities(selectedEntities);
        
        // Show the dialog
        m_dialog->show();
        m_dialog->raise();
        m_dialog->activateWindow();
    });
}