
public class IceCreamExample {
	public static void main(String[] args) {
		int icecreamScoops = Integer.parseInt(args[0]);
		iceCream(icecreamScoops);
	}
	
	private static void iceCream(int icecreamScoops) {
		if (icecreamScoops>10) 
		{
			System.out.println("Very very very super duper"
					+ "Happy"); 
		} else if (icecreamScoops==10) {
			System.out.println("Happy"); 
		}
		else if (icecreamScoops<=9 && icecreamScoops>=5) {
			int i = 1;
			if (i<=10) {
				i++;
				System.out.println("Sad");
			}
			else {
				
			}
		} else {
			int i = 1;
			if (i<=10) {
				i++;
			System.out.println("Super Sad");
			}
		}

	}
}
