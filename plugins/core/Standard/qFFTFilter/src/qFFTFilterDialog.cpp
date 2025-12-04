// qFFTFilterDialog.cpp
// Implementation of the FFT Filter dialog

#include "qFFTFilterDialog.h"
#include "ui_qFFTFilterDialog.h"

// CloudCompare includes
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <ccMainAppInterface.h>
#include <ccGLWindow.h>

// Qt includes
#include <QMessageBox>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QComboBox>
#include <QCheckBox>
#include <QLabel>
#include <QProgressDialog>
#include <QApplication>
#include <QPainter>
#include <QPainterPath>
#include <QPixmap>

// Math includes
#include <cmath>
#include <complex>
#include <algorithm>

// FFT library includes
#ifdef USE_FFTW
    #include <fftw3.h>
#else
    // KissFFT fallback
    extern "C" {
        #include "kiss_fft.h"
    }
#endif

//==============================================================================
// Constructor
//==============================================================================
qFFTFilterDialog::qFFTFilterDialog(ccMainAppInterface* app, QWidget* parent)
    : QDialog(parent)
    , m_app(app)
    , ui(new Ui::qFFTFilterDialog)
    , m_currentCloud(nullptr)
    , m_samplingFrequency(1.0)
    , m_lowFreqCutoff(0.0)
    , m_highFreqCutoff(1.0)
    , m_fftComputed(false)
    , m_downsampleFactor(100)
{
    ui->setupUi(this);
    
    // Set window title and properties
    setWindowTitle("FFT Filter");
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
    
    // Initialize UI components
    setupUI();
}

//==============================================================================
// Destructor
//==============================================================================
qFFTFilterDialog::~qFFTFilterDialog()
{
    delete ui;
}

//==============================================================================
// Setup UI components
//==============================================================================
void qFFTFilterDialog::setupUI()
{
    // Connect signals and slots
    connect(ui->computeFFTButton, &QPushButton::clicked, 
            this, &qFFTFilterDialog::onComputeFFT);
    
    connect(ui->applyFilterButton, &QPushButton::clicked,
            this, &qFFTFilterDialog::onApplyFilter);
    
    connect(ui->resetRangeButton, &QPushButton::clicked,
            this, &qFFTFilterDialog::onResetRange);
    
    connect(ui->scalarFieldComboBox, &QComboBox::currentIndexChanged,
            this, &qFFTFilterDialog::onScalarFieldChanged);
    
    connect(ui->lowFreqSlider, &QSlider::valueChanged,
            this, &qFFTFilterDialog::onLowFreqSliderChanged);
    
    connect(ui->highFreqSlider, &QSlider::valueChanged,
            this, &qFFTFilterDialog::onHighFreqSliderChanged);
    
    connect(ui->lowFreqSpinBox, &QDoubleSpinBox::valueChanged,
            this, &qFFTFilterDialog::onLowFreqSpinBoxChanged);
    
    connect(ui->highFreqSpinBox, &QDoubleSpinBox::valueChanged,
            this, &qFFTFilterDialog::onHighFreqSpinBoxChanged);
    
    connect(ui->downsampleCheckBox, &QCheckBox::stateChanged,
            this, &qFFTFilterDialog::onDownsampleChanged);
    
    // Set default values
    ui->samplingFreqSpinBox->setValue(1.0);
    ui->samplingFreqSpinBox->setMinimum(0.001);
    ui->samplingFreqSpinBox->setMaximum(1000000.0);
    ui->samplingFreqSpinBox->setDecimals(3);
    
    // Set downsample defaults
    ui->downsampleCheckBox->setChecked(true);
    ui->downsampleSpinBox->setValue(100);
    ui->downsampleSpinBox->setEnabled(true);
    
    // Initially disable filter controls until FFT is computed
    ui->lowFreqSlider->setEnabled(false);
    ui->highFreqSlider->setEnabled(false);
    ui->lowFreqSpinBox->setEnabled(false);
    ui->highFreqSpinBox->setEnabled(false);
    ui->applyFilterButton->setEnabled(false);
    ui->resetRangeButton->setEnabled(false);
}

//==============================================================================
// Update selected entities
//==============================================================================
void qFFTFilterDialog::updateSelectedEntities(const std::vector<ccHObject*>& selectedEntities)
{
    // Find the first point cloud in selection
    m_currentCloud = nullptr;
    
    for (ccHObject* entity : selectedEntities)
    {
        if (entity && entity->isA(CC_TYPES::POINT_CLOUD))
        {
            m_currentCloud = static_cast<ccPointCloud*>(entity);
            break;
        }
    }
    
    // Update UI based on current cloud
    if (m_currentCloud)
    {
        ui->cloudLabel->setText(QString("Cloud: %1").arg(m_currentCloud->getName()));
        updateScalarFieldComboBox();
        ui->computeFFTButton->setEnabled(true);
    }
    else
    {
        ui->cloudLabel->setText("No cloud selected");
        ui->scalarFieldComboBox->clear();
        ui->computeFFTButton->setEnabled(false);
        m_fftComputed = false;
    }
}

//==============================================================================
// Update scalar field combo box
//==============================================================================
void qFFTFilterDialog::updateScalarFieldComboBox()
{
    ui->scalarFieldComboBox->clear();
    
    if (!m_currentCloud)
        return;
    
    // Add all scalar fields to combo box
    unsigned sfCount = m_currentCloud->getNumberOfScalarFields();
    for (unsigned i = 0; i < sfCount; ++i)
    {
        CCCoreLib::ScalarField* sf = m_currentCloud->getScalarField(i);
        if (sf)
        {
            ui->scalarFieldComboBox->addItem(QString::fromStdString(sf->getName()), i);
        }
    }
    
    // Select the current displayed scalar field if any
    int currentSFIndex = m_currentCloud->getCurrentDisplayedScalarFieldIndex();
    if (currentSFIndex >= 0 && currentSFIndex < static_cast<int>(sfCount))
    {
        ui->scalarFieldComboBox->setCurrentIndex(currentSFIndex);
    }
}

//==============================================================================
// Extract scalar field data
//==============================================================================
std::vector<double> qFFTFilterDialog::extractScalarField(ccPointCloud* cloud, int sfIndex)
{
    std::vector<double> values;
    
    if (!cloud || sfIndex < 0)
        return values;
    
    CCCoreLib::ScalarField* sf = cloud->getScalarField(sfIndex);
    if (!sf)
        return values;
    
    // Reserve space for efficiency
    unsigned pointCount = cloud->size();
    values.reserve(pointCount);
    
    // Extract values (NaN values are kept as-is for now)
    for (unsigned i = 0; i < pointCount; ++i)
    {
        ScalarType value = sf->getValue(i);
        values.push_back(static_cast<double>(value));
    }
    
    return values;
}

//==============================================================================
// Downsample signal
//==============================================================================
std::vector<double> qFFTFilterDialog::downsampleSignal(const std::vector<double>& signal, int factor)
{
    if (factor <= 1)
        return signal;
    
    std::vector<double> downsampled;
    downsampled.reserve(signal.size() / factor + 1);
    
    for (size_t i = 0; i < signal.size(); i += factor)
    {
        downsampled.push_back(signal[i]);
    }
    
    return downsampled;
}

//==============================================================================
// Compute optimal downsampling factor
//==============================================================================
int qFFTFilterDialog::computeOptimalDownsampleFactor(size_t pointCount)
{
    // Target: ~500K to 1M points for FFT (good balance of speed and accuracy)
    const size_t targetPoints = 750000;
    
    if (pointCount <= targetPoints)
        return 1;  // No downsampling needed
    
    int factor = static_cast<int>(pointCount / targetPoints);
    
    // Round to nice numbers
    if (factor < 10)
        return factor;
    else if (factor < 100)
        return (factor / 10) * 10;  // Round to nearest 10
    else if (factor < 1000)
        return (factor / 100) * 100;  // Round to nearest 100
    else
        return (factor / 1000) * 1000;  // Round to nearest 1000
}

//==============================================================================
// Compute FFT
//==============================================================================
bool qFFTFilterDialog::computeFFT(const std::vector<double>& signal,
                                  std::vector<double>& magnitudes,
                                  std::vector<double>& frequencies)
{
    if (signal.empty())
        return false;
    
    size_t N = signal.size();
    magnitudes.resize(N / 2 + 1);  // Only need positive frequencies
    frequencies.resize(N / 2 + 1);
    
#ifdef USE_FFTW
    // Use FFTW library
    
    // Allocate input and output arrays
    double* in = fftw_alloc_real(N);
    fftw_complex* out = fftw_alloc_complex(N / 2 + 1);
    
    // Copy signal to input
    for (size_t i = 0; i < N; ++i)
        in[i] = signal[i];
    
    // Create plan and execute FFT
    fftw_plan plan = fftw_plan_dft_r2c_1d(static_cast<int>(N), in, out, FFTW_ESTIMATE);
    fftw_execute(plan);
    
    // Extract magnitudes (normalize by N)
    for (size_t i = 0; i < magnitudes.size(); ++i)
    {
        double real = out[i][0] / N;
        double imag = out[i][1] / N;
        magnitudes[i] = std::sqrt(real * real + imag * imag);
    }
    
    // Clean up
    fftw_destroy_plan(plan);
    fftw_free(in);
    fftw_free(out);
    
#else
    // Use KissFFT fallback
    
    kiss_fft_cfg cfg = kiss_fft_alloc(static_cast<int>(N), 0, nullptr, nullptr);
    
    std::vector<kiss_fft_cpx> in(N);
    std::vector<kiss_fft_cpx> out(N);
    
    // Copy signal to input (real values only)
    for (size_t i = 0; i < N; ++i)
    {
        in[i].r = static_cast<float>(signal[i]);
        in[i].i = 0.0f;
    }
    
    // Execute FFT
    kiss_fft(cfg, in.data(), out.data());
    
    // Extract magnitudes (normalize by N, only positive frequencies)
    for (size_t i = 0; i < magnitudes.size(); ++i)
    {
        double real = out[i].r / N;
        double imag = out[i].i / N;
        magnitudes[i] = std::sqrt(real * real + imag * imag);
    }
    
    // Clean up
    kiss_fft_free(cfg);
    
#endif
    
    // Compute frequency values
    double fs = m_samplingFrequency;
    for (size_t i = 0; i < frequencies.size(); ++i)
    {
        frequencies[i] = (i * fs) / N;
    }
    
    return true;
}

//==============================================================================
// Apply band-pass filter
//==============================================================================
bool qFFTFilterDialog::applyBandPassFilter(const std::vector<double>& signal,
                                           std::vector<double>& filtered,
                                           double lowFreq,
                                           double highFreq)
{
    if (signal.empty())
        return false;
    
    size_t N = signal.size();
    filtered.resize(N);
    
#ifdef USE_FFTW
    // Use FFTW with FFTW_MEASURE for optimal performance
    
    // Allocate arrays
    double* in = fftw_alloc_real(N);
    fftw_complex* freq = fftw_alloc_complex(N / 2 + 1);
    double* out = fftw_alloc_real(N);
    
    // Copy signal
    std::copy(signal.begin(), signal.end(), in);
    
    // Create OPTIMIZED plans with FFTW_MEASURE (much faster than ESTIMATE)
    // Note: FFTW_MEASURE is slower first time but much faster for actual transforms
    fftw_plan forward = fftw_plan_dft_r2c_1d(static_cast<int>(N), in, freq, FFTW_MEASURE);
    fftw_execute(forward);
    
    // Apply band-pass filter
    double fs = m_samplingFrequency;
    for (size_t i = 0; i < N / 2 + 1; ++i)
    {
        double f = (i * fs) / N;
        
        // Zero out frequencies outside the band
        if (f < lowFreq || f > highFreq)
        {
            freq[i][0] = 0.0;
            freq[i][1] = 0.0;
        }
    }
    
    // Inverse FFT
    fftw_plan backward = fftw_plan_dft_c2r_1d(static_cast<int>(N), freq, out, FFTW_MEASURE);
    fftw_execute(backward);
    
    // Copy result (normalize by N for inverse FFT)
    double normalization = 1.0 / N;
    for (size_t i = 0; i < N; ++i)
        filtered[i] = out[i] * normalization;
    
    // Clean up
    fftw_destroy_plan(forward);
    fftw_destroy_plan(backward);
    fftw_free(in);
    fftw_free(freq);
    fftw_free(out);
    
#else
    // Use KissFFT (slower fallback)
    
    kiss_fft_cfg cfg_forward = kiss_fft_alloc(static_cast<int>(N), 0, nullptr, nullptr);
    kiss_fft_cfg cfg_inverse = kiss_fft_alloc(static_cast<int>(N), 1, nullptr, nullptr);
    
    std::vector<kiss_fft_cpx> in(N);
    std::vector<kiss_fft_cpx> freq(N);
    std::vector<kiss_fft_cpx> out(N);
    
    // Copy signal
    for (size_t i = 0; i < N; ++i)
    {
        in[i].r = static_cast<float>(signal[i]);
        in[i].i = 0.0f;
    }
    
    // Forward FFT
    kiss_fft(cfg_forward, in.data(), freq.data());
    
    // Apply band-pass filter
    double fs = m_samplingFrequency;
    for (size_t i = 0; i < N; ++i)
    {
        // Handle both positive and negative frequencies
        double f;
        if (i <= N / 2)
            f = (i * fs) / N;
        else
            f = ((N - i) * fs) / N;
        
        // Zero out frequencies outside the band
        if (f < lowFreq || f > highFreq)
        {
            freq[i].r = 0.0f;
            freq[i].i = 0.0f;
        }
    }
    
    // Inverse FFT
    kiss_fft(cfg_inverse, freq.data(), out.data());
    
    // Copy result (normalize)
    double normalization = 1.0 / N;
    for (size_t i = 0; i < N; ++i)
        filtered[i] = out[i].r * normalization;
    
    // Clean up
    kiss_fft_free(cfg_forward);
    kiss_fft_free(cfg_inverse);
    
#endif
    
    return true;
}

//==============================================================================
// Slot: Downsample checkbox changed
//==============================================================================
void qFFTFilterDialog::onDownsampleChanged(int state)
{
    ui->downsampleSpinBox->setEnabled(state == Qt::Checked);
    ui->downsampleLabel->setEnabled(state == Qt::Checked);
}

//==============================================================================
// Slot: Compute FFT button clicked
//==============================================================================
void qFFTFilterDialog::onComputeFFT()
{
    if (!m_currentCloud)
    {
        QMessageBox::warning(this, "Error", "No point cloud selected");
        return;
    }
    
    // Get selected scalar field index
    int sfIndex = ui->scalarFieldComboBox->currentData().toInt();
    
    // Create progress dialog
    QProgressDialog progress("Processing...", "Cancel", 0, 100, this);
    progress.setWindowModality(Qt::WindowModal);
    progress.setMinimumDuration(0);
    
    // Extract scalar field data
    progress.setLabelText("Extracting scalar field data...");
    progress.setValue(10);
    QApplication::processEvents();
    
    m_originalSignal = extractScalarField(m_currentCloud, sfIndex);
    
    if (m_originalSignal.empty())
    {
        QMessageBox::warning(this, "Error", "Failed to extract scalar field data");
        return;
    }
    
    if (progress.wasCanceled())
        return;
    
    size_t originalSize = m_originalSignal.size();
    
    // Auto-compute optimal downsampling
    int optimalFactor = computeOptimalDownsampleFactor(originalSize);
    
    // Get downsampling settings
    bool useDownsampling = ui->downsampleCheckBox->isChecked();
    m_downsampleFactor = useDownsampling ? ui->downsampleSpinBox->value() : 1;
    
    // Suggest optimal if significantly different
    if (useDownsampling && optimalFactor != m_downsampleFactor && optimalFactor > 1)
    {
        auto reply = QMessageBox::question(this, "Optimization Suggestion",
            QString("For %1M points, recommended downsample factor: %2\n"
                    "Current setting: %3\n\n"
                    "Use recommended value?")
            .arg(originalSize / 1000000.0, 0, 'f', 1)
            .arg(optimalFactor)
            .arg(m_downsampleFactor),
            QMessageBox::Yes | QMessageBox::No);
        
        if (reply == QMessageBox::Yes)
        {
            m_downsampleFactor = optimalFactor;
            ui->downsampleSpinBox->setValue(optimalFactor);
        }
    }
    
    // Downsample if requested
    progress.setLabelText("Downsampling...");
    progress.setValue(30);
    QApplication::processEvents();
    
    std::vector<double> signalForFFT;
    if (m_downsampleFactor > 1)
    {
        signalForFFT = downsampleSignal(m_originalSignal, m_downsampleFactor);
    }
    else
    {
        signalForFFT = m_originalSignal;
    }
    
    if (progress.wasCanceled())
        return;
    
    size_t fftSize = signalForFFT.size();
    
    // Adjust sampling frequency
    m_samplingFrequency = ui->samplingFreqSpinBox->value() / m_downsampleFactor;
    
    // Compute FFT
    progress.setLabelText(QString("Computing FFT on %1M points...").arg(fftSize / 1000000.0, 0, 'f', 2));
    progress.setValue(50);
    QApplication::processEvents();
    
    if (!computeFFT(signalForFFT, m_fftMagnitudes, m_fftFrequencies))
    {
        QMessageBox::warning(this, "Error", "FFT computation failed");
        return;
    }
    
    if (progress.wasCanceled())
        return;
    
    // Update plot
    progress.setLabelText("Updating display...");
    progress.setValue(90);
    QApplication::processEvents();
    
    updatePlot();
    updateSliderRanges();
    
    // Enable filter controls
    ui->lowFreqSlider->setEnabled(true);
    ui->highFreqSlider->setEnabled(true);
    ui->lowFreqSpinBox->setEnabled(true);
    ui->highFreqSpinBox->setEnabled(true);
    ui->applyFilterButton->setEnabled(true);
    ui->resetRangeButton->setEnabled(true);
    
    m_fftComputed = true;
    
    progress.setValue(100);
    
    QMessageBox::information(this, "Success",
        QString("FFT completed!\n\n"
                "Original: %1M points\n"
                "FFT computed on: %2M points\n"
                "Downsampling: %3x\n"
                "Frequency resolution: %4 Hz")
        .arg(originalSize / 1000000.0, 0, 'f', 1)
        .arg(fftSize / 1000000.0, 0, 'f', 2)
        .arg(m_downsampleFactor)
        .arg(m_fftFrequencies.size() > 1 ? m_fftFrequencies[1] - m_fftFrequencies[0] : 0.0, 0, 'e', 3));
}

//==============================================================================
// Update plot
//==============================================================================
void qFFTFilterDialog::updatePlot()
{
    if (m_fftMagnitudes.empty() || m_fftFrequencies.empty())
        return;
    
    // Find max magnitude for scaling
    double maxMag = *std::max_element(m_fftMagnitudes.begin(), m_fftMagnitudes.end());
    
    // Update plot info label
    ui->plotInfoLabel->setText(
        QString("FFT Spectrum | Max magnitude: %1 | Frequency range: 0 - %2 Hz")
        .arg(maxMag, 0, 'e', 3)
        .arg(m_fftFrequencies.back(), 0, 'f', 3)
    );
    
    // Draw the FFT spectrum on the plot widget
    QWidget* plotWidget = ui->plotWidget;
    QPixmap pixmap(plotWidget->size());
    pixmap.fill(Qt::white);
    
    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // Set up drawing area (leave margins for axes)
    int margin = 40;
    int plotWidth = pixmap.width() - 2 * margin;
    int plotHeight = pixmap.height() - 2 * margin;
    
    if (plotWidth <= 0 || plotHeight <= 0)
        return;
    
    // Draw axes
    painter.setPen(QPen(Qt::black, 2));
    painter.drawLine(margin, margin, margin, margin + plotHeight);  // Y axis
    painter.drawLine(margin, margin + plotHeight, margin + plotWidth, margin + plotHeight);  // X axis
    
    // Draw axis labels
    QFont font = painter.font();
    font.setPointSize(8);
    painter.setFont(font);
    
    painter.drawText(QRect(0, margin + plotHeight + 5, pixmap.width(), 30), 
                     Qt::AlignCenter, "Frequency (Hz)");
    
    painter.save();
    painter.translate(15, margin + plotHeight / 2);
    painter.rotate(-90);
    painter.drawText(QRect(-100, -10, 200, 20), Qt::AlignCenter, "Magnitude");
    painter.restore();
    
    // Draw FFT spectrum
    if (m_fftMagnitudes.size() > 1 && maxMag > 0)
    {
        painter.setPen(QPen(QColor(0, 100, 200), 1));
        
        // Draw line connecting points
        QPainterPath path;
        bool firstPoint = true;
        
        for (size_t i = 0; i < m_fftMagnitudes.size(); ++i)
        {
            double freq = m_fftFrequencies[i];
            double mag = m_fftMagnitudes[i];
            
            // Map to plot coordinates
            int x = margin + static_cast<int>((freq / m_fftFrequencies.back()) * plotWidth);
            int y = margin + plotHeight - static_cast<int>((mag / maxMag) * plotHeight);
            
            if (firstPoint)
            {
                path.moveTo(x, y);
                firstPoint = false;
            }
            else
            {
                path.lineTo(x, y);
            }
        }
        
        painter.drawPath(path);
        
        // Draw cutoff lines if filter is set
        if (m_lowFreqCutoff > 0 || m_highFreqCutoff < m_fftFrequencies.back())
        {
            painter.setPen(QPen(Qt::red, 2, Qt::DashLine));
            
            // Low cutoff line
            int xLow = margin + static_cast<int>((m_lowFreqCutoff / m_fftFrequencies.back()) * plotWidth);
            painter.drawLine(xLow, margin, xLow, margin + plotHeight);
            
            // High cutoff line
            int xHigh = margin + static_cast<int>((m_highFreqCutoff / m_fftFrequencies.back()) * plotWidth);
            painter.drawLine(xHigh, margin, xHigh, margin + plotHeight);
        }
    }
    
    painter.end();
    
    // Set the pixmap as the widget's background
    QPalette palette;
    palette.setBrush(QPalette::Window, QBrush(pixmap));
    plotWidget->setPalette(palette);
    plotWidget->setAutoFillBackground(true);
    plotWidget->update();
}

//==============================================================================
// Update slider ranges
//==============================================================================
void qFFTFilterDialog::updateSliderRanges()
{
    if (m_fftFrequencies.empty())
        return;
    
    double maxFreq = m_fftFrequencies.back();
    
    // Set spinbox ranges
    ui->lowFreqSpinBox->setRange(0.0, maxFreq);
    ui->highFreqSpinBox->setRange(0.0, maxFreq);
    
    // Set default values (full range)
    ui->lowFreqSpinBox->setValue(0.0);
    ui->highFreqSpinBox->setValue(maxFreq);
    
    m_lowFreqCutoff = 0.0;
    m_highFreqCutoff = maxFreq;
    
    // Set slider ranges (use 1000 steps for smooth control)
    ui->lowFreqSlider->setRange(0, 1000);
    ui->highFreqSlider->setRange(0, 1000);
    ui->lowFreqSlider->setValue(0);
    ui->highFreqSlider->setValue(1000);
}

//==============================================================================
// Slot handlers for sliders and spinboxes
//==============================================================================
void qFFTFilterDialog::onLowFreqSliderChanged(int value)
{
    if (m_fftFrequencies.empty())
        return;
    
    double maxFreq = m_fftFrequencies.back();
    double freq = (value / 1000.0) * maxFreq;
    
    // Prevent crossing
    if (freq > m_highFreqCutoff)
        freq = m_highFreqCutoff;
    
    m_lowFreqCutoff = freq;
    ui->lowFreqSpinBox->blockSignals(true);
    ui->lowFreqSpinBox->setValue(freq);
    ui->lowFreqSpinBox->blockSignals(false);
    
    updatePlot();  // Update visual cutoff indicators
}

void qFFTFilterDialog::onHighFreqSliderChanged(int value)
{
    if (m_fftFrequencies.empty())
        return;
    
    double maxFreq = m_fftFrequencies.back();
    double freq = (value / 1000.0) * maxFreq;
    
    // Prevent crossing
    if (freq < m_lowFreqCutoff)
        freq = m_lowFreqCutoff;
    
    m_highFreqCutoff = freq;
    ui->highFreqSpinBox->blockSignals(true);
    ui->highFreqSpinBox->setValue(freq);
    ui->highFreqSpinBox->blockSignals(false);
    
    updatePlot();
}

void qFFTFilterDialog::onLowFreqSpinBoxChanged(double value)
{
    if (m_fftFrequencies.empty())
        return;
    
    // Prevent crossing
    if (value > m_highFreqCutoff)
        value = m_highFreqCutoff;
    
    m_lowFreqCutoff = value;
    
    double maxFreq = m_fftFrequencies.back();
    int sliderValue = static_cast<int>((value / maxFreq) * 1000);
    
    ui->lowFreqSlider->blockSignals(true);
    ui->lowFreqSlider->setValue(sliderValue);
    ui->lowFreqSlider->blockSignals(false);
    
    updatePlot();
}

void qFFTFilterDialog::onHighFreqSpinBoxChanged(double value)
{
    if (m_fftFrequencies.empty())
        return;
    
    // Prevent crossing
    if (value < m_lowFreqCutoff)
        value = m_lowFreqCutoff;
    
    m_highFreqCutoff = value;
    
    double maxFreq = m_fftFrequencies.back();
    int sliderValue = static_cast<int>((value / maxFreq) * 1000);
    
    ui->highFreqSlider->blockSignals(true);
    ui->highFreqSlider->setValue(sliderValue);
    ui->highFreqSlider->blockSignals(false);
    
    updatePlot();
}

//==============================================================================
// Slot: Apply filter button clicked
//==============================================================================
void qFFTFilterDialog::onApplyFilter()
{
    if (!m_fftComputed || m_originalSignal.empty())
    {
        QMessageBox::warning(this, "Error", "Please compute FFT first");
        return;
    }
    
    // Show progress
    QProgressDialog progress("Applying filter to full signal...", "Cancel", 0, 100, this);
    progress.setWindowModality(Qt::WindowModal);
    progress.setMinimumDuration(0);
    progress.setValue(10);
    QApplication::processEvents();
    
    // Adjust frequency cutoffs for full signal sampling rate
    // (Remember: FFT was computed on downsampled signal, but filter applies to full signal)
    double fullSamplingFreq = ui->samplingFreqSpinBox->value();
    
    progress.setLabelText(QString("Filtering %1M points...").arg(m_originalSignal.size() / 1000000.0, 0, 'f', 1));
    progress.setValue(30);
    QApplication::processEvents();
    
    if (progress.wasCanceled())
        return;
    
    // OPTIMIZED: Use the full sampling frequency (not downsampled)
    double originalSamplingFreq = m_samplingFrequency;
    m_samplingFrequency = fullSamplingFreq;
    
    // Apply band-pass filter
    std::vector<double> filtered;
    if (!applyBandPassFilter(m_originalSignal, filtered, m_lowFreqCutoff, m_highFreqCutoff))
    {
        m_samplingFrequency = originalSamplingFreq;  // Restore
        QMessageBox::warning(this, "Error", "Filter application failed");
        return;
    }
    
    // Restore original sampling frequency
    m_samplingFrequency = originalSamplingFreq;
    
    if (progress.wasCanceled())
        return;
    
    progress.setValue(80);
    progress.setLabelText("Creating scalar field...");
    QApplication::processEvents();
    
    // Create new scalar field or update existing one
    QString sfName = ui->scalarFieldComboBox->currentText() + "_filtered";
    
    if (ui->createNewSFCheckBox->isChecked())
    {
        // Create new scalar field
        if (!createFilteredScalarField(m_currentCloud, filtered, sfName))
        {
            QMessageBox::warning(this, "Error", "Failed to create new scalar field");
            return;
        }
    }
    else
    {
        // Update current scalar field
        int sfIndex = ui->scalarFieldComboBox->currentData().toInt();
        CCCoreLib::ScalarField* sf = m_currentCloud->getScalarField(sfIndex);
        
        if (sf && filtered.size() == m_currentCloud->size())
        {
            for (size_t i = 0; i < filtered.size(); ++i)
            {
                sf->setValue(static_cast<unsigned>(i), static_cast<ScalarType>(filtered[i]));
            }
            sf->computeMinAndMax();
        }
    }
    
    progress.setValue(100);
    
    // Update CloudCompare display
    m_currentCloud->setCurrentDisplayedScalarField(static_cast<int>(m_currentCloud->getNumberOfScalarFields()) - 1);
    m_currentCloud->showSF(true);
    
    if (m_app)
    {
        m_app->refreshAll();
        m_app->updateUI();
    }
    
    QMessageBox::information(this, "Success", 
        QString("Filter applied successfully\n"
                "Filtered %1M points\n"
                "Band: %2 - %3 Hz")
        .arg(m_originalSignal.size() / 1000000.0, 0, 'f', 1)
        .arg(m_lowFreqCutoff, 0, 'f', 3)
        .arg(m_highFreqCutoff, 0, 'f', 3));
}

//==============================================================================
// Create filtered scalar field
//==============================================================================
bool qFFTFilterDialog::createFilteredScalarField(ccPointCloud* cloud,
                                                 const std::vector<double>& data,
                                                 const QString& name)
{
    if (!cloud || data.empty() || data.size() != cloud->size())
        return false;
    
    // Create new scalar field
    int sfIndex = cloud->addScalarField(name.toStdString().c_str());
    if (sfIndex < 0)
        return false;
    
    CCCoreLib::ScalarField* sf = cloud->getScalarField(sfIndex);
    if (!sf)
        return false;
    
    // Fill with filtered data
    for (size_t i = 0; i < data.size(); ++i)
    {
        sf->setValue(static_cast<unsigned>(i), static_cast<ScalarType>(data[i]));
    }
    
    // Compute min/max for proper display
    sf->computeMinAndMax();
    
    return true;
}

//==============================================================================
// Slot: Reset range button clicked
//==============================================================================
void qFFTFilterDialog::onResetRange()
{
    if (m_fftFrequencies.empty())
        return;
    
    double maxFreq = m_fftFrequencies.back();
    
    ui->lowFreqSpinBox->setValue(0.0);
    ui->highFreqSpinBox->setValue(maxFreq);
    
    m_lowFreqCutoff = 0.0;
    m_highFreqCutoff = maxFreq;
}

//==============================================================================
// Slot: Scalar field changed
//==============================================================================
void qFFTFilterDialog::onScalarFieldChanged(int index)
{
    // Reset FFT when changing scalar field
    m_fftComputed = false;
    m_fftMagnitudes.clear();
    m_fftFrequencies.clear();
    m_originalSignal.clear();
    
    ui->lowFreqSlider->setEnabled(false);
    ui->highFreqSlider->setEnabled(false);
    ui->lowFreqSpinBox->setEnabled(false);
    ui->highFreqSpinBox->setEnabled(false);
    ui->applyFilterButton->setEnabled(false);
    ui->resetRangeButton->setEnabled(false);
}