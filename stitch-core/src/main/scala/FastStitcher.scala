import boofcv.abst.feature.associate.AssociateDescription
import boofcv.abst.feature.detect.interest.ConfigFastHessian
import boofcv.factory.feature.associate.FactoryAssociation
import boofcv.factory.feature.detdesc.FactoryDetectDescribe
import boofcv.factory.geo.{ConfigRansac, FactoryMultiViewRobust}
import boofcv.io.image.ConvertBufferedImage
import boofcv.struct.feature.BrightFeature
import boofcv.struct.geo.AssociatedPair
import boofcv.struct.image.GrayF32
import georegression.struct.homography.Homography2D_F64
import georegression.struct.point.Point2D_F64
import java.awt.Dimension
import java.awt.image.BufferedImage
import org.ddogleg.fitting.modelset.ModelMatcher
import org.ddogleg.struct.FastQueue
import scala.collection.mutable

/**
 * Created by ashwin on 3/12/17.
 */
class FastStitcher {



//  def stitch(
//    size: Dimension,
//    frames: Seq[BufferedImage]
//  ): Seq[BufferedImage] = {
//    // Convert all images into a compatible BoofCV format.
//    val input = frames.map(ConvertBufferedImage.convertFromSingle(_, null, classOf[GrayF32]))
//
//    // Find interest points for all frames.
//    val hessian = new ConfigFastHessian(1, 2, 200, 1, 9, 4, 4)
//    val ipoints = input.map(describe(FactoryDetectDescribe.surfStable(hessian, null, null, classOf[GrayF32])))
//
//    // Find homographies between adjacent frames.
//    val scorer = FactoryAssociation.scoreEuclidean(classOf[BrightFeature], true)
//    val ransac = new ConfigRansac(60, 3)
//
//    val forward = ipoints.sliding(2).map { case (a, b) =>
//      val associator = FactoryAssociation.greedy(scorer, 2, true)
//      val matcher = FactoryMultiViewRobust.homographyRansac(null, ransac)
//      homography(associator, matcher)(a, b)
//    }
//
//    // Calculate backwards homographies by taking the inverse of the forward.
//    val backward = forward.map(_.invert(null))
//
//
//  }
//
//  def homography(
//    associator: AssociateDescription[BrightFeature],
//    matcher: ModelMatcher[Homography2D_F64, AssociatedPair]
//  )(
//    ipointsA: (Seq[Point2D_F64], FastQueue[BrightFeature]),
//    ipointsB: (Seq[Point2D_F64], FastQueue[BrightFeature])
//  ): Homography2D_F64 = {
//    associator.setSource(ipointsA._2)
//    associator.setDestination(ipointsB._2)
//    associator.associate()
//    val matches = associator.getMatches
//
//    // Construct an association between points.
//    val pairs = mutable.Buffer.empty[AssociatedPair]
//    (0 until matches.size).map(matches.get).foreach { m =>
//      pairs += new AssociatedPair(ipointsA._1(m.src), ipointsB._1(m.dst), false)
//    }
//
//    // Attempt to construct the homography.
//    if(!matcher.process(pairs.asJava))
//      throw new RuntimeException("Unable to determine homography.")
//    else
//      matcher.getModelParameters.copy()
//  }

}
