
/**
 * Class: Pair
 * 
 * This class implements a pair object that has two objects: a left and a right.
 * They can be of two different classes. 
 * 
 * @author yhindy
 *
 * @param <L>
 * @param <R>
 */
public class Pair<L,R> {
		private final L left;
		private final R right;
		
		public Pair(L left, R right) {
			this.left = left;
			this.right = right;
		}
		
		public L getLeft() {
			return left;
		}
		
		public R getRight() {
			return right;
		}
		
		@Override
		public boolean equals(Object o) {
		    if (!(o instanceof Pair)) return false;
		    Pair<?,?> pairo = (Pair<?,?>) o;
		    return this.left.equals(pairo.getLeft()) &&
		            this.right.equals(pairo.getRight());
		  }
	}
	