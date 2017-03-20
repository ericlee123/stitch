package com.stitch.core

import boofcv.abst.feature.associate.AssociateDescription
import boofcv.abst.feature.detdesc.DetectDescribePoint
import boofcv.abst.feature.detect.interest.ConfigFastHessian
import boofcv.alg.descriptor.UtilFeature
import boofcv.alg.distort.PixelTransformHomography_F32
import boofcv.alg.distort.impl.DistortSupport
import boofcv.core.image.border.BorderType
import boofcv.factory.feature.associate.FactoryAssociation
import boofcv.factory.feature.detdesc.FactoryDetectDescribe
import boofcv.factory.geo.{ConfigRansac, FactoryMultiViewRobust}
import boofcv.factory.interpolate.FactoryInterpolation
import boofcv.io.image.ConvertBufferedImage
import boofcv.struct.feature.BrightFeature
import boofcv.struct.geo.AssociatedPair
import boofcv.struct.image.GrayF32
import georegression.struct.homography.Homography2D_F64
import georegression.struct.point.Point2D_F64
import java.awt.image.BufferedImage
import org.ddogleg.fitting.modelset.ModelMatcher
import org.ddogleg.struct.FastQueue
import scala.collection.mutable
import scala.collection.JavaConverters._
import java.awt.Dimension
import java.io.File
import javax.imageio.ImageIO

object Stitcher {

  def main(args: Array[String]): Unit = {

    // Input parameters.
    val size   = new Dimension(1920, 1080)
    val video  = "stitch-assets/balcony.mp4"
//    val dir    = "/Users/ashwin/Downloads/stitcher" // Ashwin
    val dir = "stitch-assets/video-frames" // Eric
    val output = "stitch-assets/balcony-stitched.mp4"
    val look   = 300
    val jump   = 25

    // Extract frames from the video using ffmpeg.
    ffmpeg(video, dir + "/frame-%07d.png")
    println("done splitting video into individual frames")

    // calculating all incremental homographies at once
    val frameFiles = new File(dir).listFiles(f => f.getName.matches("frame-[0-9]+.png")).sortWith(_.getName < _.getName)
    val numFrames = frameFiles.length
    val homographies = computeHomographies(frameFiles)
    println("done computing homographies")

    // generate the frames
    (0 to numFrames - 1).par foreach { i =>
      println("stitching frame " + (i + 1))
      ImageIO.write(stitchFrame(frameFiles, homographies, size, look, jump, i), "png", new File(dir + "/stitched-%07d.png".format(i + 1)))
    }
    println("done stitching frames")

    // use ffmpeg to generate a video
    ffmpeg(dir + "/stitched-%07d.png", output)
    println("done generating stitched video")
  }

  /**
   * Calls ffmpeg to either (1) extract frames from a video or (2) generate a video from frames.
   * Each function is called by swapping the input and output parameters.
   *
   * @param input Input data.
   * @param output Output location.
   */
  def ffmpeg(input: String, output: String): Unit = {
    new ProcessBuilder("mkdir", "-p", output).start().waitFor() // TODO: huh? (kind of messy I guess)
    new ProcessBuilder("rm", "-r", output).start().waitFor()
    val pb = new ProcessBuilder("ffmpeg", "-i", input, output)
    pb.start().waitFor() // TODO: check for error code (make folder first)
  }

  /**
    * Compute homographies a given sequences of frames while staying within the JVM heap limit.
    * The sequence of homographies is structured such that homographies(i) is a mapping of
    * frames(i+1) onto frames(i).
    * @param frameFiles Sequence of frames.
    * @return Sequence of homographies.
    */
  def computeHomographies(frameFiles: Seq[File]): Seq[Homography2D_F64] = {
    val numFrames = frameFiles.length
    val frameSpace = frameFiles.length
    val capacity = 50 // TODO: use Runtime to determine how many images to read in at a time

    var homographies = Seq[Homography2D_F64]()
    var nextFrame = 0
    while (nextFrame < numFrames) {
      println("homography progress " + nextFrame + " / " + (numFrames - 1))
      val frames = frameFiles
        .slice(nextFrame, (nextFrame + capacity) min (numFrames - 1))
        .map(ImageIO.read)

      // TODO: find a way to parallelize
      homographies = homographies ++ transform(frames) // what is the best way to concat sequences? use another data structure?
      nextFrame += capacity - 1
    }
    homographies // is there any way to do this cleanly? instead of having this "return" expression at the end
  }

  def stitchFrame(
    frameFiles: Seq[File],
    homographies: Seq[Homography2D_F64],
    size: Dimension,
    look: Int,
    jump: Int,
    i: Int
  ): BufferedImage = {

    val numFrames = frameFiles.length

    // Setup the rendering toolchain. Put it in here because multithreading.
    val model = new PixelTransformHomography_F32
    val interpolater = FactoryInterpolation.bilinearPixelS(classOf[GrayF32], BorderType.ZERO)
    val distortion = DistortSupport.createDistortPL(classOf[GrayF32], model, interpolater, false)
    distortion.setRenderAll(false)

    val currentFrame = ImageIO.read(frameFiles(i))
    var canvas = new BufferedImage(size.width * 2, size.height * 2, currentFrame.getType)
    draw(canvas, currentFrame, size)

    if (i > 0) {
      val backOne = ImageIO.read(frameFiles(i - 1))
      var backT = transform(canvas, backOne)

      homographies.zipWithIndex.slice(0 max (i - look), i - 2).reverse.foreach { case (h, j) =>

        backT = backT.concat(h.invert(null), null)

        if ((i - j) % jump == 0 || j == 0) {
          val otherFrame = ImageIO.read(frameFiles(j))
          val colorCurrent = ConvertBufferedImage.convertFromMulti(canvas, null, true, classOf[GrayF32])
          val colorOther = ConvertBufferedImage.convertFromMulti(otherFrame, null, true, classOf[GrayF32])

          model.set(backT)
          distortion.apply(colorOther, colorCurrent)
          canvas = ConvertBufferedImage.convertTo(colorCurrent, new BufferedImage(colorCurrent.width, colorCurrent.height, canvas.getType), true)
        }
      }
      draw(canvas, currentFrame, size)
    }
    if (i < numFrames - 1) { // TODO: make this go in correct order, from furthest to closest
    val forwardOne = ImageIO.read(frameFiles(i + 1))
      var forwardT = transform(canvas, forwardOne)
      homographies.zipWithIndex.slice(i + 1, (i + look) min numFrames).foreach { case (h, j) =>
        if ((j - i) % jump == 0 || j == numFrames - 1) {
          val otherFrame = ImageIO.read(frameFiles(j))
          val colorCurrent = ConvertBufferedImage.convertFromMulti(canvas, null, true, classOf[GrayF32])
          val colorOther = ConvertBufferedImage.convertFromMulti(otherFrame, null, true, classOf[GrayF32])

          model.set(forwardT)
          distortion.apply(colorOther, colorCurrent)
          canvas = ConvertBufferedImage.convertTo(colorCurrent, new BufferedImage(colorCurrent.width, colorCurrent.height, canvas.getType), true)
        }
        forwardT = forwardT.concat(h, null)
      }
      draw(canvas, currentFrame, size)
    }

    val x = (canvas.getWidth - size.width) / 2
    val y = (canvas.getHeight - size.height) / 2
    canvas.getSubimage(x, y, size.width, size.height)
  }

  /**
   * Scales the image to the provided dimensions, while preserving aspect ratio, and then draws the
   * resized image in the center of the provided canvas.
   *
   * @param canvas Image to draw on.
   * @param image Image to draw.
   * @param size Scaled dimensions of image.
   */
  def draw(
    canvas: BufferedImage,
    image: BufferedImage,
    size: Dimension
  ): Unit = {
    // Calculate the width, height, and aspect ratio of the image.
    val w = image.getWidth
    val h = image.getHeight
    val r = w / h.toDouble
    val graphics = canvas.getGraphics

    // Resize the image while preserving aspect ratio.
    if (h > w) {
      val width = (size.getHeight * r).toInt
      val dx = (canvas.getWidth - width) / 2
      val dy = (canvas.getHeight - size.height) / 2
      graphics.drawImage(image, dx, dy, dx + width, dy + size.height, 0, 0, w, h, null)
    } else {
      val height = (size.getWidth / r).toInt
      val dx = (canvas.getWidth - size.width) / 2
      val dy = (canvas.getHeight - height) / 2
      graphics.drawImage(image, dx, dy, dx + size.width, dy + size.height, 0, 0, w, h, null)
    }

    // Cleanup the graphics instance.
    graphics.dispose()
  }

  /**
    * Takes in a sequence of consecutive frames and compute the homographies between each frame t and t+1.
    * @param frames Frames from video.
    * @return Sequence of homographies that transform frame t to t-1.
    */
  def transform(
    frames: Seq[BufferedImage]
  ): Seq[Homography2D_F64] = {
    val descriptor = FactoryDetectDescribe.surfStable(new ConfigFastHessian(1, 2, 200, 1, 9, 4, 4), null, null, classOf[GrayF32])
    val associator = FactoryAssociation.greedy(FactoryAssociation.scoreEuclidean(classOf[BrightFeature], true), 2, true)
    val matcher = FactoryMultiViewRobust.homographyRansac(null, new ConfigRansac(60, 3))

//    // This gets weird errors (trying to parallelize)
//    frames
//      .map(ConvertBufferedImage.convertFromSingle(_, null, classOf[GrayF32]))
//      .sliding(2)
//      .toSeq // wtf is going on here???
//      .par
//      .map(x => homography(descriptor, associator, matcher)(x.head, x.last))
//      .seq

    frames
      .map(ConvertBufferedImage.convertFromSingle(_, null, classOf[GrayF32]))
      .sliding(2)
      .map(x => homography(descriptor, associator, matcher)(x.head, x.last))
      .toSeq
  }

  /**
    * Returns a transformation that maps other onto main.
    * @param main Main image.
    * @param other Other image.
    */
  def transform(
    main: BufferedImage,
    other: BufferedImage
  ): Homography2D_F64 = {
    val descriptor = FactoryDetectDescribe.surfStable(new ConfigFastHessian(1, 2, 200, 1, 9, 4, 4), null, null, classOf[GrayF32])
    val associator = FactoryAssociation.greedy(FactoryAssociation.scoreEuclidean(classOf[BrightFeature], true), 2, true)
    val matcher = FactoryMultiViewRobust.homographyRansac(null, new ConfigRansac(60, 3))

    homography(descriptor, associator, matcher)(ConvertBufferedImage.convertFromSingle(main, null, classOf[GrayF32]), ConvertBufferedImage.convertFromSingle(other, null, classOf[GrayF32]))
  }

  /**
   * Returns a homography from image B to A. Utilizes the provided detection algorithm to find
   * interest points, the provided association algorithm to group them, and the provided matching
   * model to determine the homography.
   *
   * @param descriptor Detection algorithm.
   * @param associator Association algorithm.
   * @param matcher Matching algorithm.
   * @param inputA First input image.
   * @param inputB Second input image.
   * @return Homography from image B to image A.
   */
  def homography(
    descriptor: DetectDescribePoint[GrayF32, BrightFeature],
    associator: AssociateDescription[BrightFeature],
    matcher: ModelMatcher[Homography2D_F64, AssociatedPair]
  )(
    inputA: GrayF32,
    inputB: GrayF32
  ): Homography2D_F64 = {
    // Locate a matching between the interest points of both images.
    val (pointsA, descA) = describe(descriptor)(inputA)
    val (pointsB, descB) = describe(descriptor)(inputB)

    associator.setSource(descA)
    associator.setDestination(descB)
    associator.associate()
    val matches = associator.getMatches

    // Construct an association between points.
    val pairs = mutable.Buffer.empty[AssociatedPair]
    (0 until matches.size).map(matches.get).foreach { m =>
      pairs += new AssociatedPair(pointsA(m.src), pointsB(m.dst), false)
    }

    // Attempt to construct the homography.
    if(!matcher.process(pairs.asJava))
      throw new RuntimeException("Unable to determine homography.")
    else
      matcher.getModelParameters.copy()
  }

  /**
   * Returns a tuple containing the location and descriptions of the various interest points in the
   * provided image. Interest points are found using the provided detection algorithm. Location and
   * description indices correspond.
   *
   * @param detector Detection algorithm.
   * @param input Input image.
   * @return Location and description of interest points.
   */
  def describe(
    detector: DetectDescribePoint[GrayF32, BrightFeature]
  )(
    input: GrayF32
  ): (Seq[Point2D_F64], FastQueue[BrightFeature]) = {
    val points = mutable.Buffer.empty[Point2D_F64]
    val descriptors = UtilFeature.createQueue(detector, 100)

    detector.detect(input)
    (0 until detector.getNumberOfFeatures) foreach { i =>
      points += detector.getLocation(i).copy()
      descriptors.grow().setTo(detector.getDescription(i))
    }

    (points, descriptors)
  }

}