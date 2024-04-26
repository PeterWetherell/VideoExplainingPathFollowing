package localizerDemonstrationVideo;

import localizer.ConstVelLocalizer;
import localizer.Localizer;

public class ConstVelMath {
	public static void main(String[] args) {
		Localizer l = new ConstVelLocalizer();
		LocalizerDemostrationVideo vid = new LocalizerDemostrationVideo(l);
	}
}
