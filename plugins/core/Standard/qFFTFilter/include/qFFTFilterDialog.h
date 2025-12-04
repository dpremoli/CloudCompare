// qFFTFilterDialog.h
// The main UI dialog for FFT filtering operations

#ifndef Q_FFT_FILTER_DIALOG_HEADER
#define Q_FFT_FILTER_DIALOG_HEADER

#include <QDialog>
#include <vector>

// Forward declarations
class ccMainAppInterface;
class ccPointCloud;
class ccHObject;
class QComboBox;
class QDoubleSpinBox;
class QPushButton;
class QSlider;
class QCheckBox;
class QLabel;

namespace Ui {
    class qFFTFilterDialog;
}

//! FFT Filter Dialog
class qFFTFilterDialog : public QDialog
{
    Q_OBJECT

public:
    explicit qFFTFilterDialog(ccMainAppInterface* app, QWidget* parent = nullptr);
    ~qFFTFilterDialog() override;
    
    void updateSelectedEntities(const std::vector<ccHObject*>& selectedEntities);

protected slots:
    void onComputeFFT();
    void onApplyFilter();
    void onResetRange();
    void onScalarFieldChanged(int index);
    void onLowFreqSliderChanged(int value);
    void onHighFreqSliderChanged(int value);
    void onLowFreqSpinBoxChanged(double value);
    void onHighFreqSpinBoxChanged(double value);
    void onDownsampleChanged(int state);

private:
    void setupUI();
    void updateScalarFieldComboBox();
    
    std::vector<double> extractScalarField(ccPointCloud* cloud, int sfIndex);
    std::vector<double> downsampleSignal(const std::vector<double>& signal, int factor);
    int computeOptimalDownsampleFactor(size_t pointCount);
    
    bool computeFFT(const std::vector<double>& signal,
                    std::vector<double>& magnitudes,
                    std::vector<double>& frequencies);
    
    bool applyBandPassFilter(const std::vector<double>& signal,
                            std::vector<double>& filtered,
                            double lowFreq,
                            double highFreq);
    
    void updatePlot();
    void updateSliderRanges();
    
    bool createFilteredScalarField(ccPointCloud* cloud,
                                   const std::vector<double>& data,
                                   const QString& name);

private:
    ccMainAppInterface* m_app;
    Ui::qFFTFilterDialog* ui;
    ccPointCloud* m_currentCloud;
    
    std::vector<double> m_fftMagnitudes;
    std::vector<double> m_fftFrequencies;
    std::vector<double> m_originalSignal;
    
    double m_samplingFrequency;
    double m_lowFreqCutoff;
    double m_highFreqCutoff;
    
    bool m_fftComputed;
    int m_downsampleFactor;
};

#endif // Q_FFT_FILTER_DIALOG_HEADER