package localizerDemonstrationVideo;

import localizer.ConstAccelLocalizer;
import localizer.Localizer;

public class ConstAccelMath {
	public static void main(String[] args) {
		Localizer l = new ConstAccelLocalizer();
		LocalizerDemostrationVideo vid = new LocalizerDemostrationVideo(l);
	}
}
