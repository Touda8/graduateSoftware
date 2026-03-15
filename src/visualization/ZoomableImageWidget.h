#pragma once

#include <QWidget>
#include <QPixmap>
#include <QPointF>

namespace tp {

/// QWidget subclass that displays a QPixmap with aspect-ratio-preserving fit
/// and supports mouse-wheel zoom + click-drag pan.
class ZoomableImageWidget : public QWidget {
    Q_OBJECT

public:
    explicit ZoomableImageWidget(QWidget* parent = nullptr);

    /// Set image to display (thread-safe if called from GUI thread)
    void setImage(const QPixmap& pix);

    /// Clear displayed image
    void clearImage();

    /// Reset zoom/pan to fit-in-view
    void resetView();

    QSize sizeHint() const override { return {400, 300}; }
    QSize minimumSizeHint() const override { return {100, 80}; }

protected:
    void paintEvent(QPaintEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    QPixmap pixmap_;
    double zoom_ = 1.0;
    QPointF offset_{0, 0};      // pan offset in widget coordinates
    QPointF dragStart_;
    QPointF offsetAtDragStart_;
    bool dragging_ = false;
};

} // namespace tp
