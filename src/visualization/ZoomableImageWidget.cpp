#include "visualization/ZoomableImageWidget.h"

#include <QPainter>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QResizeEvent>
#include <algorithm>
#include <cmath>

namespace tp {

ZoomableImageWidget::ZoomableImageWidget(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(100, 80);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setMouseTracking(false);
}

void ZoomableImageWidget::setImage(const QPixmap& pix)
{
    image_ = pix.toImage();
    resetView();
    update();
}

void ZoomableImageWidget::setImage(const QImage& img)
{
    image_ = img;
    resetView();
    update();
}

void ZoomableImageWidget::clearImage()
{
    image_ = QImage();
    zoom_ = 1.0;
    offset_ = {0, 0};
    update();
}

void ZoomableImageWidget::resetView()
{
    zoom_ = 1.0;
    offset_ = {0, 0};
}

void ZoomableImageWidget::paintEvent(QPaintEvent* /*event*/)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::SmoothPixmapTransform);
    painter.fillRect(rect(), palette().window());

    // 统一边框
    painter.setPen(QPen(QColor(190, 190, 190), 1));
    painter.drawRect(rect().adjusted(0, 0, -1, -1));

    if (image_.isNull()) return;

    // Compute base scale to fit image in widget with aspect ratio preserved
    double wRatio = static_cast<double>(width()) / image_.width();
    double hRatio = static_cast<double>(height()) / image_.height();
    double baseScale = std::min(wRatio, hRatio);
    double totalScale = baseScale * zoom_;

    double drawW = image_.width() * totalScale;
    double drawH = image_.height() * totalScale;

    // Center + pan offset
    double cx = (width() - drawW) / 2.0 + offset_.x();
    double cy = (height() - drawH) / 2.0 + offset_.y();

    QRectF target(cx, cy, drawW, drawH);
    painter.drawImage(target, image_, QRectF(image_.rect()));
}

void ZoomableImageWidget::wheelEvent(QWheelEvent* event)
{
    if (image_.isNull()) {
        event->ignore();
        return;
    }

    double delta = event->angleDelta().y() / 120.0;
    double factor = std::pow(1.15, delta);

    double oldZoom = zoom_;
    zoom_ *= factor;
    zoom_ = std::clamp(zoom_, 0.1, 50.0);

    // Zoom toward mouse position
    double realFactor = zoom_ / oldZoom;
#if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
    QPointF mousePos = event->position();
#else
    QPointF mousePos = event->posF();
#endif
    QPointF widgetCenter(width() / 2.0, height() / 2.0);
    QPointF mouseFromCenter = mousePos - widgetCenter - offset_;
    offset_ -= mouseFromCenter * (realFactor - 1.0);

    update();
    event->accept();
}

void ZoomableImageWidget::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton && !image_.isNull()) {
        dragging_ = true;
        dragStart_ = event->pos();
        offsetAtDragStart_ = offset_;
        setCursor(Qt::ClosedHandCursor);
        event->accept();
    } else {
        QWidget::mousePressEvent(event);
    }
}

void ZoomableImageWidget::mouseMoveEvent(QMouseEvent* event)
{
    if (dragging_) {
        QPointF delta = QPointF(event->pos()) - dragStart_;
        offset_ = offsetAtDragStart_ + delta;
        update();
        event->accept();
    } else {
        QWidget::mouseMoveEvent(event);
    }
}

void ZoomableImageWidget::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton && dragging_) {
        dragging_ = false;
        setCursor(Qt::ArrowCursor);
        event->accept();
    } else {
        QWidget::mouseReleaseEvent(event);
    }
}

void ZoomableImageWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    // Keep offset proportional on resize (optional: just repaint)
    update();
}

} // namespace tp
