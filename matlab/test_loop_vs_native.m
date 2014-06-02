function test_loop_vs_native()
% test_loop_vs_native Illustrates the cost of Matlab loops.

% (C) Copyright Renaud Detry 2013.
% Distributed under the GNU General Public License.
% (See http://www.gnu.org/copyleft/gpl.html)

  img1 = uint8([]);
  img1(:,:,1) = repmat(uint8(0:511)/2, 512, 1);
  img1(:,:,2) = repmat(uint8(0:511)'/2, 1, 512);
  img1(:,:,3) = repmat(uint8(511:-1:0), 512, 1);
  % imshow(img1);

  img2 = uint8([]);
  img3 = uint8([]);

  tic;
  for i = 1:size(img1, 1),
    for j = 1:size(img1, 2),
      for k = 1:size(img1, 3),
        img2(size(img1,1)-i+1,j,k) = img1(i,j,k);
      end
    end
  end
  t1 = toc;

  tic;
  img3 = flipdim(img1, 1);
  t2 = toc;

  fprintf('Flipping %ix%i image with a loop  : %f seconds\n', size(img1, 1), size(img1, 2), t1);
  fprintf('Flipping %ix%i image with flipdim : %f seconds\n', size(img1, 1), size(img1, 2), t2);

  % Result on an i5 with Matlab R2010a:
  % Flipping 512x512 image with a loop  : 0.320369 seconds
  % Flipping 512x512 image with flipdim : 0.001194 seconds

  % Result on an i5 with Matlab R2013a:
  % Flipping 512x512 image with a loop  : 1.202010 seconds
  % Flipping 512x512 image with flipdim : 0.001205 seconds

end
